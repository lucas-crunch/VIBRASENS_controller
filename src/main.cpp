#include <Arduino.h>
#include <M5Unified.h>
#include <M5GFX.h>
#include "M5UnitScroll.h"
#include "M5StickCPlus2.h"
#include "network_espnow.h"

M5UnitScroll scroll;

#include "ClosedCube_TCA9548A.h"
ClosedCube::Wired::TCA9548A tca9548a;
#define PaHub_I2C_ADDRESS (0x70)

#include "EEPROM.h"
#include "Preferences.h"
#include <nvs_flash.h>


#define IDLE 0
#define CHANNEL_CHOICE 1
int state = IDLE;

long timer_holding = 0;

Preferences preferences;

int encoder_value_1 = 0;
int encoder_value_2 = 0;
int last_encoder_value_1 = 0;
int last_encoder_value_2 = 0;
int max_value_encoder_1 = 10;
int max_value_encoder_2 = 2000;
int encoder_value_1_sent = 0;
int encoder_value_2_sent = 0;

uint8_t btn_status_1 = 0;
uint8_t btn_status_2 = 0;
uint8_t last_btn_status_1 = 0;
uint8_t last_btn_status_2 = 0;

bool change = false;

long timerScreenSleep = 0;
long intervalScreenSleep = 10000; // 5 seconds

void initScreen() {
    M5.Display.setRotation(0);
    M5.Display.clear(LIGHTGREY);
    M5.Display.setTextColor(BLACK);
    M5.Display.setTextDatum(CC_DATUM);
    M5.Display.setFont(&Orbitron_Light_24);
    M5.Display.setTextSize(1);
}

void screenBackground() {
    M5.Display.drawString("Intensite", 135 / 2, 20);
    M5.Display.drawString("Temps", 135 / 2, 100);
    M5.Display.drawString("Batterie", 135 / 2, 190);
}

void upgradeScreen() {
    M5.Display.fillRect(0, 35, 135, 30, LIGHTGREY);
    M5.Display.fillRect(0, 115, 135, 30, LIGHTGREY);
    M5.Display.fillRect(0, 205, 135, 30, LIGHTGREY);
    M5.Display.drawString(String(encoder_value_1_sent), 135 / 2, 50);
    M5.Display.drawString(String(encoder_value_2_sent) + " ms", 135 / 2, 130);
    M5.Display.drawString(String(StickCP2.Power.getBatteryLevel()), 135 / 2, 220);
}

void upgradeScreenChannel() {
    M5.Display.fillRect(0, 35, 135, 30, YELLOW);
    M5.Display.fillRect(0, 205, 135, 30, YELLOW);
    M5.Display.drawString(String(channel_number), 135 / 2, 50);
    M5.Display.drawString(String(StickCP2.Power.getBatteryLevel()), 135 / 2, 220);
}

void screenBackgroundChannel() {
    M5.Display.drawString("Channel", 135 / 2, 20);
    M5.Display.drawString("Batterie", 135 / 2, 190);
}

ESP_NOW_Broadcast_Peer broadcast_peer(channel_number, WIFI_IF_STA, NULL);

void setup() {
    M5.begin();

    preferences.begin("my-app", false);
    encoder_value_1_sent = preferences.getInt("encoder1", 0);
    encoder_value_2_sent = preferences.getInt("encoder2", 0);
    channel_number = preferences.getInt("channel", 6);



    initScreen();
    screenBackground();
    upgradeScreen();

    Wire.begin(32, 33); // SDA: GPIO21, SCL: GPIO22
    Serial.begin(115200);
    Serial.println("START");
    Serial.println(channel_number);

    // Create a broadcast peer object
    initESPNOW();

    scroll.begin(&Wire, SCROLL_ADDR, 26, 32, 400000U);
    tca9548a.address(PaHub_I2C_ADDRESS); // Set the I2C address.  设置I2C地址

    // write your initialization code here
}

void loop() {
    M5.update();

    switch (state) {
        case IDLE:
            tca9548a.selectChannel(2);
            encoder_value_1 = scroll.getIncEncoderValue();
            btn_status_1 = scroll.getButtonStatus();

            tca9548a.selectChannel(3);
            encoder_value_2 = scroll.getIncEncoderValue();
            btn_status_2 = scroll.getButtonStatus();


            if (encoder_value_1 != 0) {
                timerScreenSleep = millis();
                if (encoder_value_1 == -1) {
                    if (encoder_value_1_sent >= max_value_encoder_1) {
                        encoder_value_1_sent = max_value_encoder_1;
                    } else {
                        encoder_value_1_sent++;
                    }
                } else if (encoder_value_1 == 1) {
                    if (encoder_value_1_sent <= 0) {
                        encoder_value_1_sent = 0;
                    } else {
                        encoder_value_1_sent--;
                    }
                }
                Serial.print("encoder_value_1_sent: ");
                Serial.println(encoder_value_1_sent);
                preferences.putInt("encoder1", encoder_value_1_sent);
                upgradeScreen();
            }

            if (encoder_value_2 != 0) {
                timerScreenSleep = millis();
                if (encoder_value_2 == -1) {
                    if (encoder_value_2_sent >= max_value_encoder_2) {
                        encoder_value_2_sent = max_value_encoder_2;
                    } else {
                        encoder_value_2_sent = encoder_value_2_sent + 100;
                    }
                } else if (encoder_value_2 == 1) {
                    if (encoder_value_2_sent <= 0) {
                        encoder_value_2_sent = 0;
                    } else {
                        encoder_value_2_sent = encoder_value_2_sent - 100;
                    }
                }
                Serial.print("encoder_value_2_sent: ");
                Serial.println(encoder_value_2_sent);
                preferences.putInt("encoder2", encoder_value_2_sent);
                upgradeScreen();
            }

            if (btn_status_1 != last_btn_status_1) {
                Serial.print("btn_status_1: ");
                Serial.println(btn_status_1);
                last_btn_status_1 = btn_status_1;
            }

            if (btn_status_2 != last_btn_status_2) {
                Serial.print("btn_status_2: ");
                Serial.println(btn_status_2);
                last_btn_status_2 = btn_status_2;
            }


            if (M5.BtnA.wasPressed()) {
                M5.Display.wakeup();
                timerScreenSleep = millis();
                initScreen();
                screenBackground();
                upgradeScreen();

                Serial.println("BtnA was pressed.");
                change = true;

                encoder_msg.encoder1 = encoder_value_1_sent;
                encoder_msg.encoder2 = encoder_value_2_sent;
                if (!br_peer->send_message((uint8_t *) &encoder_msg, sizeof(encoder_msg))) {
                    Serial.println("Failed to broadcast message");
                }
            }


            if (M5.BtnPWR.wasClicked()) {
                M5.Display.clear(YELLOW);
                screenBackgroundChannel();
                upgradeScreenChannel();
                Serial.println("BtnC was pressed.");
                state = CHANNEL_CHOICE;
                delay(500);
            }

            break;

        case CHANNEL_CHOICE:
            if (M5.BtnPWR.wasClicked()) {
                M5.Display.clear(LIGHTGREY);
                screenBackground();
                upgradeScreen();
                state = IDLE;
                delay(500);
            }

            if (M5.BtnB.wasClicked()) {
                timerScreenSleep = millis();
                channel_number++;
                if (channel_number > 8) {
                    channel_number = 1;
                }
                upgradeScreenChannel();
            }

            if (M5.BtnA.wasClicked()) {
                preferences.putInt("channel", channel_number);
                M5.Display.drawString("Saved", 135 / 2, 100);
                ESP.restart();
            }
            break;
    }

    if (millis() - timerScreenSleep > intervalScreenSleep) {
        timerScreenSleep = millis();
        M5.Display.sleep();
        preferences.putInt("encoder1", encoder_value_1_sent);
        preferences.putInt("encoder2", encoder_value_2_sent);

        esp_sleep_enable_ext0_wakeup((gpio_num_t) GPIO_NUM_37, LOW);
        gpio_hold_en((gpio_num_t) GPIO_NUM_4);
        gpio_deep_sleep_hold_en();
        esp_deep_sleep_start();
    }

    delay(30);
/*
    if (M5.BtnPWR.isHolding()) {

    }
    */


    // write your code here
}
