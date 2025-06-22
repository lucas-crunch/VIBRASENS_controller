//
// Created by lromary on 17/06/2025.
//

#ifndef NETWORK_ESPNOW_H
#define NETWORK_ESPNOW_H

#endif //NETWORK_ESPNOW_H

#include "WiFi.h"
#include "ESP32_NOW.h"
#include <esp_mac.h>  // For the MAC2STR and MACSTR macros

/* Definitions */

uint8_t channel_number = 4;

typedef struct
{
    int encoder1 = 0;
    int encoder2 = 0;
} __attribute__((packed)) esp_now_encoder_data_t;
esp_now_encoder_data_t encoder_msg;

/* Classes */

// Creating a new class that inherits from the ESP_NOW_Peer class is required.

class ESP_NOW_Broadcast_Peer : public ESP_NOW_Peer {
public:
    // Constructor of the class using the broadcast address
    ESP_NOW_Broadcast_Peer(uint8_t channel, wifi_interface_t iface, const uint8_t *lmk) : ESP_NOW_Peer(ESP_NOW.BROADCAST_ADDR, channel, iface, lmk) {}

    // Destructor of the class
    ~ESP_NOW_Broadcast_Peer() {
        remove();
    }

    // Function to properly initialize the ESP-NOW and register the broadcast peer
    bool begin() {
        if (!ESP_NOW.begin() || !add()) {
            log_e("Failed to initialize ESP-NOW or register the broadcast peer");
            return false;
        }
        return true;
    }

    // Function to send a message to all devices within the network
    bool send_message(const uint8_t *data, size_t len) {
        if (!send(data, len)) {
            log_e("Failed to broadcast message");
            return false;
        }
        return true;
    }
    void onSent(bool success)
    {
        bool broadcast = memcmp(addr(), ESP_NOW.BROADCAST_ADDR, ESP_NOW_ETH_ALEN) == 0;
        if (broadcast)
        {
            log_i("Broadcast message reported as sent %s", success ? "successfully" : "unsuccessfully");
        }
        else
        {
            log_i("Unicast message reported as sent %s to peer " MACSTR, success ? "successfully" : "unsuccessfully", MAC2STR(addr()));
        }
    }

};

ESP_NOW_Broadcast_Peer *br_peer = nullptr;

void initESPNOW() {

    WiFi.mode(WIFI_STA);

    WiFi.setChannel(channel_number,WIFI_SECOND_CHAN_NONE);
    while (!WiFi.STA.started()) {
        delay(100);
    }

    Serial.println("ESP-NOW Example - Broadcast Master");
    Serial.println("Wi-Fi parameters:");
    Serial.println("  Mode: STA");
    Serial.println("  MAC Address: " + WiFi.macAddress());
    Serial.printf("  Channel: %d\n", channel_number);

    //ESP_NOW_Broadcast_Peer broadcast_peer(channel_number, WIFI_IF_STA, NULL);
    br_peer = new ESP_NOW_Broadcast_Peer(channel_number, WIFI_IF_STA, NULL);
    // Register the broadcast peer
    if (!br_peer->begin()) {
        Serial.println("Failed to initialize broadcast peer");
        Serial.println("Reebooting in 5 seconds...");
        delay(5000);
        ESP.restart();
    }

    Serial.println("Setup complete. Broadcasting messages every 5 seconds.");
}



/* Main */