#include <Arduino.h>
#include "RBControl_manager.hpp"
#include <Servo.h>
#include <Wire.h>
#include "time.hpp"
#include <stdint.h>
#include "stopwatch.hpp"
#include "nvs_flash.h"
#include "BluetoothSerial.h"

// #include <HardwareSerial.h>
#include <ODriveArduino.h>

#define OTOCNY_MOTOR  rb::MotorId::M1 // motor pro otaceni servoruky
HardwareSerial odriveSerial(1);
ODriveArduino odrive(odriveSerial);

rb::Manager& rbc() 
{
    static rb::Manager m(false);  // ve výchozím stavu se motory po puštění tlačítka vypínají, false zařídí, že pojedou, dokud nedostanou další pokyn 
    return m;
}

Servo servo0, servo1, servo2, servo3; 
int position_servo0 = 90;
int position_servo1 = 90;
int position_servo2 = 90;
int position_servo3 = 90;
int krok_serva = 2;


void setup() {
    Serial.begin(115200);
    // odriveSerial.begin(115200, SERIAL_8N1, 13, 15);

    // Serial.println("Starting");
    // odrive.run_state(0, ODriveArduino::AXIS_STATE_FULL_CALIBRATION_SEQUENCE, true);
    // odrive.run_state(1, ODriveArduino::AXIS_STATE_FULL_CALIBRATION_SEQUENCE, true);
    // odrive.run_state(0, ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL, true);
    // odrive.run_state(1, ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL, true);


    // Serial.println("Going up");
    // for (int i = 0; i != 1000; i += 100) {
    //     odrive.SetPosition(0, i);
    //     odrive.SetPosition(1, i);
    //     delay(1);
    // }
    // Serial.println("Going down");
    // for (int i = 1000; i != 0; i -= 100) {
    //     odrive.SetPosition(0, i);
    //     odrive.SetPosition(1, i);
    //     delay(1);
    // }
    servo0.attach(27);
    servo0.write(position_servo0);
    servo1.attach(26);
    servo1.write(position_servo1);
    servo2.attach(33);
    servo2.write(position_servo2);
    servo3.attach(32);
    servo3.write(position_servo3);

}

void loop() {
  // Serial.println("Going up");
  // for (int i = 0; i != 1000; i += 100) {
  //   odrive.SetPosition(0, i);
  //   odrive.SetPosition(1, i);
  //   delay(1);
  // }
  // Serial.println("Going down");
  // for (int i = 1000; i != 0; i -= 100) {
  //   odrive.SetPosition(0, i);
  //   odrive.SetPosition(1, i);
  //   delay(1);
  // }
    if(Serial.available()) {
        char c = Serial.read();
        switch(c) {
            case 'q':
                if (position_servo3 >= 5)  position_servo3 = position_servo3 - krok_serva;               
                servo3.write(position_servo3);
                Serial.write(" 3: "); 
                Serial.print(position_servo3);
                break;
            case 'e':
                if (position_servo3 <= 175)  position_servo3 = position_servo3 + krok_serva;               
                servo3.write(position_servo3);
                Serial.write(" 3: "); 
                Serial.print(position_servo3);
                break;
           case 'a':
                if (position_servo2 >= 5)  position_servo2 = position_servo2 - krok_serva;               
                servo2.write(position_servo2);
                Serial.write(" 2: "); 
                Serial.print(position_servo2);
                break;
            case 'd':
                if (position_servo2 <= 175)  position_servo2 = position_servo2 + krok_serva;               
                servo2.write(position_servo2);
                Serial.write(" 2: "); 
                Serial.print(position_servo2);
                break;
            case 'y':
                if (position_servo1 >= 5)  position_servo1 = position_servo1 - krok_serva;               
                servo1.write(position_servo1);
                Serial.write(" 1: "); 
                Serial.print(position_servo1);
                break;
            case 'c':
                if (position_servo1 <= 175)  position_servo1 = position_servo1 + krok_serva;               
                servo1.write(position_servo1);
                Serial.write(" 1: "); 
                Serial.print(position_servo1);
                break;
           case 't':
                if (position_servo0 >= 5)  position_servo0 = position_servo0 - krok_serva;               
                servo0.write(position_servo0);
                Serial.write(" 0: "); 
                Serial.print(position_servo0);
                break;
            case 'u':
                if (position_servo0 <= 175)  position_servo0 = position_servo0 + krok_serva;               
                servo0.write(position_servo0);
                Serial.write(" 0: "); 
                Serial.print(position_servo0);
                break;

            default:
                Serial.write(c);
                break;
        }
    }
    // delay(10);
    // Serial.println(millis());


}


// Servo servo; 

// int servo_open = 100;
// int servo_close = 180;
// int position_servo = 100; // pro postupne krokovani serva pro kalibraci 
// int power_motor = 192;
// int otacka = 235; // pocet tiku na otacku 
// int ctverec = 250; // pocet tiku na ctverec - Praha
// int zatoc = 280;  // pocet tiku na zatoceni o 90 stupnu
// static const uint32_t i2c_freq = 400000;
// bool L_G_light = false; // pro blikani zelene LED - indikuje, ze deska funguje 

