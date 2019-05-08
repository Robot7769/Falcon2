#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <string>

namespace wifi
{

#include "credentials.hpp"

template <class Stream>
bool connect(Stream& debug) {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);
    uint8_t cnt = 0;
    bool connected = false;
    std::string msg;
    while (cnt++ < 4) {
        debug.println("\nConnecting...");
        int status = WiFi.begin(SSID, PSWD);
        for (uint8_t i = 0; i < 20; ++i) {
            delay(100);
            switch (status) {
                case WL_CONNECTED:
                    debug.println("Connected");
                    cnt = 255;
                    i = 254;
                    connected = true;
                    break;
                case WL_NO_SHIELD:
                    msg = ("Heh. This is really weird, it looks as if your ESP did not have WiFi.");
                    cnt = 255;
                    i = 254;
                    break;
                case WL_IDLE_STATUS:
                    break;
                case WL_NO_SSID_AVAIL:
                    msg = ("I see no WiFi networks.");
                    break;
                case WL_SCAN_COMPLETED:
                    msg = ("Hmm, WiFi scan is done. But I did not start any scan.");
                    break;
                case WL_CONNECT_FAILED:
                    msg = ("Connection failed.");
                    break;
                case WL_CONNECTION_LOST:
                    msg = ("Connection lost. Never mind it has not been started yet.");
                    break;
                case WL_DISCONNECTED:
                    msg = ("Disconnected.");
                    break;
            }
            if (msg != "Disconnected.") {
                debug.print("Status #");
                debug.print(i);
                debug.print(": ");
                debug.println(msg.c_str());
            }
            status = WiFi.status();
        }
    }
    return connected;
}

bool connect() { return connect(Serial); }

} // namespace wifi
