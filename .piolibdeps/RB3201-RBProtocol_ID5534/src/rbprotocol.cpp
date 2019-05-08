#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <memory>
#include <stdarg.h>
#include <sys/time.h>

#include "esp_log.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "rbprotocol.h"
#include "rbjson.h"

using namespace rbjson;

static const char* TAG = "RbProtocol";

#define MUST_ARRIVE_TIMER_PERIOD 50
#define MUST_ARRIVE_ATTEMPTS 40

static int diff_ms(timeval& t1, timeval& t2) {
    return (((t1.tv_sec - t2.tv_sec) * 1000000) + 
            (t1.tv_usec - t2.tv_usec))/1000;
}

/// @private
class SemaphoreHolder {
public:
    SemaphoreHolder(SemaphoreHandle_t mu) : m_mutex(mu) {
        xSemaphoreTake(mu, portMAX_DELAY);
    }
    ~SemaphoreHolder() {
        xSemaphoreGive(m_mutex);
    }
private:
    SemaphoreHandle_t m_mutex;
};

namespace rb {

Protocol::Protocol(const char *owner, const char *name, const char *description,
    std::function<void(const std::string& cmd, rbjson::Object* pkt)> callback) {
    m_owner = owner;
    m_name = name;
    m_desc = description;
    m_callback = callback;

    m_socket = -1;
    m_mutex = xSemaphoreCreateMutex();

    m_read_counter = 0;
    m_write_counter = 0;

    m_mustarrive_e = 0;
    m_mustarrive_f = 0;
    m_mustarrive_mutex = xSemaphoreCreateMutex();

    memset(&m_possessed_addr, 0, sizeof(struct sockaddr_in));
}

Protocol::~Protocol() {
    stop();
}

void Protocol::start(int port) {
    SemaphoreHolder mu(m_mutex);

    if(m_socket != -1) {
        ESP_LOGE(TAG, "start() when already started.");
        return;
    }
    
    m_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP); 
    if(m_socket == -1) {
        ESP_LOGE(TAG, "failed to create socket: %s", strerror(errno));
        return;
    }

    int enable = 1;
    if(setsockopt(m_socket, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) == -1) {
        ESP_LOGE(TAG, "failed to set SO_REUSEADDR: %s", strerror(errno));
        close(m_socket);
        m_socket = -1;
        return;
    }

    int flags = fcntl(m_socket, F_GETFL,0);
    if(flags < 0) {
        ESP_LOGE(TAG, "failed to F_GETFL: %s", strerror(errno));
        close(m_socket);
        m_socket = -1;
        return;
    }

    if(fcntl(m_socket, F_SETFL, flags | O_NONBLOCK) < 0) {
        ESP_LOGE(TAG, "failed to set O_NONBLOCK: %s", strerror(errno));
        close(m_socket);
        m_socket = -1;
        return;
    }

    struct sockaddr_in addr_bind;
    memset(&addr_bind, 0, sizeof(addr_bind));
    addr_bind.sin_family = AF_INET;
    addr_bind.sin_port = htons(port);
    addr_bind.sin_addr.s_addr = htonl(INADDR_ANY);
     
    //bind socket to port
    if(bind(m_socket, (struct sockaddr*)&addr_bind, sizeof(addr_bind)) == -1)
    {
        ESP_LOGE(TAG, "failed to bind socket: %s", strerror(errno));
        close(m_socket);
        m_socket = -1;
        return;
    }

    xTaskCreate(&Protocol::read_task_trampoline, "rbctrl_reader", 4096, this, 5, NULL);
}

void Protocol::stop() {
    SemaphoreHolder mu(m_mutex);
    if(m_socket != -1) {
        close(m_socket);
        m_socket = -1;
    }
}

bool Protocol::is_possessed() const {
    xSemaphoreTake(m_mutex, portMAX_DELAY);
    bool res = m_possessed_addr.sin_port != 0;
    xSemaphoreGive(m_mutex);
    return res;
}

void Protocol::send_mustarrive(const char *cmd, Object *params) {
    if(params == NULL) {
        params = new Object();
    }

    struct sockaddr_in addr;
    xSemaphoreTake(m_mutex, portMAX_DELAY);
    if(m_possessed_addr.sin_port == 0) {
        ESP_LOGW(TAG, "can't send, the device was not possessed yet.");
        xSemaphoreGive(m_mutex);
        return;
    }
    memcpy(&addr, &m_possessed_addr, sizeof(struct sockaddr_in));
    xSemaphoreGive(m_mutex);

    MustArrive mr;
    mr.pkt = params;
    mr.attempts = 0;

    params->set("c", cmd);

    xSemaphoreTake(m_mustarrive_mutex, portMAX_DELAY);
    mr.id = ++m_mustarrive_e;
    params->set("e", mr.id);
    m_mustarrive_queue.push_back(mr);
    xSemaphoreGive(m_mustarrive_mutex);

    send(&addr, params);
}

void Protocol::send(const char *cmd, Object *obj) {
    struct sockaddr_in addr;
    xSemaphoreTake(m_mutex, portMAX_DELAY);
    if(m_possessed_addr.sin_port == 0) {
        ESP_LOGW(TAG, "can't send, the device was not possessed yet.");
        xSemaphoreGive(m_mutex);
        return;
    }
    memcpy(&addr, &m_possessed_addr, sizeof(struct sockaddr_in));
    xSemaphoreGive(m_mutex);

    send(&addr, cmd, obj);
}

void Protocol::send(struct sockaddr_in *addr, const char *cmd, Object *obj) {
    std::unique_ptr<Object> autoptr;
    if(obj == NULL) {
        obj = new Object();
        autoptr.reset(obj);
    }

    obj->set("c", new String(cmd));
    send(addr, obj);
}

void Protocol::send(struct sockaddr_in *addr, Object *obj) {
    xSemaphoreTake(m_mutex, portMAX_DELAY);
    int n = m_write_counter++;
    xSemaphoreGive(m_mutex);

    obj->set("n", new Number(n));
    auto str = obj->str();
    send(addr, str.c_str(), str.size());
}

void Protocol::send(struct sockaddr_in *addr, const char *buf) {
    send(addr, buf, strlen(buf));
}

void Protocol::send(struct sockaddr_in *addr, const char *buf, size_t size) {
    xSemaphoreTake(m_mutex, portMAX_DELAY);
    int socket = m_socket;
    xSemaphoreGive(m_mutex);

    if(m_socket == -1) {
        return;
    }

    int res = ::sendto(socket, buf, size, 0, (struct sockaddr*)addr, sizeof(struct sockaddr_in));
    while(res < 0 && (errno = EAGAIN || errno == EWOULDBLOCK)) {
        vTaskDelay(1);
        res = ::sendto(socket, buf, size, 0, (struct sockaddr*)addr, sizeof(struct sockaddr_in));
    }

    if(res < 0) {
        ESP_LOGE(TAG, "failed to send message %.*s: %s", size, buf, strerror(errno));
    }
}

void Protocol::send_log(const char *fmt, ...) {
    char buf[512];

    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    Object *pkt = new Object();
    pkt->set("msg", buf);
    send_mustarrive("log", pkt);
}

void Protocol::read_task_trampoline(void *ctrl) {
    ((Protocol*)ctrl)->read_task();
}

void Protocol::read_task() {
    xSemaphoreTake(m_mutex, portMAX_DELAY);
    int socket = m_socket;
    xSemaphoreGive(m_mutex);

    static const int bufsize = 4096;

    struct sockaddr_in addr_recv;
    socklen_t addrlen = sizeof(struct sockaddr_in);
    char *buf = new char[bufsize]; 
    ssize_t read;

    uint32_t mustarrive_timer = MUST_ARRIVE_TIMER_PERIOD;
    struct timeval tv_last, tv_now;
    gettimeofday(&tv_last, NULL);

    while(true) {
        do {
            read = recvfrom(socket, buf, bufsize-1, MSG_DONTWAIT, (struct sockaddr*)&addr_recv, &addrlen);
            if(read > 0) {
                buf[read] = 0;
                handle_msg(&addr_recv, buf, read);
            } else if(errno == EBADF || errno == ENOTCONN) {
                ESP_LOGI(TAG, "read routine closing due to error");
                return;
            }
        } while(read > 0);

        gettimeofday(&tv_now, NULL);
        uint32_t diff = diff_ms(tv_now, tv_last);
        memcpy(&tv_last, &tv_now, sizeof(struct timeval));

        if(mustarrive_timer <= diff) {
            xSemaphoreTake(m_mustarrive_mutex, portMAX_DELAY);
            if(m_mustarrive_queue.size() != 0) {
                struct sockaddr_in addr;
                xSemaphoreTake(m_mutex, portMAX_DELAY);
                memcpy(&addr, &m_possessed_addr, sizeof(struct sockaddr_in));
                xSemaphoreGive(m_mutex);
                for(auto itr = m_mustarrive_queue.begin(); itr != m_mustarrive_queue.end(); ) {
                    send(&addr, (*itr).pkt);
                    if((*itr).attempts != -1 && ++(*itr).attempts >= MUST_ARRIVE_ATTEMPTS) {
                        delete (*itr).pkt;
                        itr = m_mustarrive_queue.erase(itr);
                    } else {
                        ++itr;
                    }
                }
            }
            xSemaphoreGive(m_mustarrive_mutex);
            mustarrive_timer = MUST_ARRIVE_TIMER_PERIOD;
        } else {
            mustarrive_timer -= diff;
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void Protocol::handle_msg(struct sockaddr_in *addr, char *buf, ssize_t size) {
    std::unique_ptr<Object> pkt(parse(buf, size));
    if(!pkt) {
        ESP_LOGE(TAG, "failed to parse the packet's json");
        return;
    }

    auto cmd = pkt->getString("c");
    
    ESP_LOGD(TAG, "Got command: %s %.*s", cmd.c_str(), size, buf);

    if(cmd == "discover") {
        std::unique_ptr<Object> res(new Object());
        res->set("c", "found");
        res->set("owner", m_owner);
        res->set("name", m_name);
        res->set("desc", m_desc);

        const auto str = res->str();
        send(addr, str.c_str(), str.size());
        return;
    }

    if(!pkt->contains("n")) {
        ESP_LOGE(TAG, "packet does not have counter %s", buf);
        return;
    }

    int counter = pkt->getInt("n");
    if(counter == -1) {
        m_read_counter = 0;
        xSemaphoreTake(m_mutex, portMAX_DELAY);
        m_write_counter = 0;
        xSemaphoreGive(m_mutex);
    } else if(counter < m_read_counter && m_read_counter - counter < 300) {
        return;
    } else {
        m_read_counter = counter;
    }

    if(pkt->contains("f")) {
        send(addr, pkt.get());

        if(cmd == "possess") {
            xSemaphoreTake(m_mutex, portMAX_DELAY);
            bool different = memcmp(&m_possessed_addr, addr, sizeof(struct sockaddr_in));
            if(different) {
                memcpy(&m_possessed_addr, addr, sizeof(struct sockaddr_in));
                m_mustarrive_e = 0;
                m_mustarrive_f = 0;
            }
            xSemaphoreGive(m_mutex);

            xSemaphoreTake(m_mustarrive_mutex, portMAX_DELAY);
            for(auto it : m_mustarrive_queue) {
                delete it.pkt;
            }
            m_mustarrive_queue.clear();
            xSemaphoreGive(m_mustarrive_mutex);
        }

        int f = pkt->getInt("f");
        if(f <= m_mustarrive_f) {
            return;
        } else {
            m_mustarrive_f = f;
        }
    } else if(pkt->contains("e")) {
        int e = pkt->getInt("e");
        xSemaphoreTake(m_mustarrive_mutex, portMAX_DELAY);
        for(auto itr = m_mustarrive_queue.begin(); itr != m_mustarrive_queue.end(); ++itr) {
            if((*itr).id == e) {
                delete (*itr).pkt;
                m_mustarrive_queue.erase(itr);
                break;
            }
        }
        xSemaphoreGive(m_mustarrive_mutex);
        return;
    }

    if(cmd == "possess") {   
        ESP_LOGI(TAG, "We are possessed!");
        send_log("The device %s has been possessed!\n", m_name);
    } else if(m_callback != NULL) {
        m_callback(cmd, pkt.get());
    }
}


}; // namespace rb