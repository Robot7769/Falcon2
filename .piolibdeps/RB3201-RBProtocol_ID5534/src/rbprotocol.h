#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <string>
#include <vector>
#include <sys/socket.h>
#include <functional>

#include "rbjson.h"

namespace rb {

#define RBPROTOCOL_PORT 42424 //!< The default RBProtocol port

#define RBPROTOCOL_AXIS_MIN (-32767) //!< Minimal value of axes in "joy" command
#define RBPROTOCOL_AXIS_MAX (32767)  //!< Maximal value of axes in "joy" command

class Protocol;

/**
 * \brief Class that manages the RBProtocol communication
 */
class Protocol {
public:
    /**
     * The onPacketReceivedCallback is called when a packet arrives.
     * It runs on a separate task, only single packet is processed at a time.
     */
    Protocol(const char *owner, const char *name, const char *description,
        std::function<void(const std::string& cmd, rbjson::Object* pkt)> callback = nullptr);
    ~Protocol();

    void start(int port = RBPROTOCOL_PORT); //!< Start listening for UDP packets on port
    void stop(); //!< Stop listening

    /**
     * \brief Send command cmd with params, without making sure it arrives.
     * 
     * If you pass the params object, you are responsible for its deletion.
     */
    void send(const char *cmd, rbjson::Object *params = NULL);

    /**
     * \brief Send command cmd with params and make sure it arrives.
     * 
     * If you pass the params object, it has to be heap-allocated and 
     * RbProtocol becomes its owner - you MUST NOT delete it.
     */
    void send_mustarrive(const char *cmd, rbjson::Object *params = NULL);

    void send_log(const char *fmt, ...); //!< Send a message to the android app
    void send_log(const char *str); //!< Send a message to the android app

    bool is_possessed() const; //!< Returns true of the device is possessed (somebody connected to it) 

private:
    struct MustArrive {
        rbjson::Object *pkt;
        uint32_t id;
        int16_t attempts;
    };

    static void read_task_trampoline(void *ctrl);
    void read_task();

    void send(struct sockaddr_in *addr, const char *command, rbjson::Object *obj);
    void send(struct sockaddr_in *addr, rbjson::Object *obj);
    void send(struct sockaddr_in *addr, const char *buf);
    void send(struct sockaddr_in *addr, const char *buf, size_t size);

    void handle_msg(struct sockaddr_in *addr, char *buf, ssize_t size);

    const char *m_owner;
    const char *m_name;
    const char *m_desc;

    std::function<void(const std::string& cmd, rbjson::Object* pkt)> m_callback;

    int m_socket;
    int m_read_counter;
    int m_write_counter;
    struct sockaddr_in m_possessed_addr;
    SemaphoreHandle_t m_mutex;

    uint32_t m_mustarrive_e;
    uint32_t m_mustarrive_f;
    std::vector<MustArrive> m_mustarrive_queue;
    SemaphoreHandle_t m_mustarrive_mutex;
};

};