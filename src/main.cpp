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

using rb::LED_GREEN;
#define OTOCNY_MOTOR  rb::MotorId::M1 // motor pro otaceni servoruky
HardwareSerial odriveSerial(1);
ODriveArduino odrive(odriveSerial);

rb::Manager& rbc() 
{
    static rb::Manager m(false);  // ve výchozím stavu se motory po puštění tlačítka vypínají, false zařídí, že pojedou, dokud nedostanou další pokyn 
    return m;
}

Servo servo0, servo1, servo2, servo3; 
int position_servo0 = 60;
int position_servo1 = 90;
int position_servo2 = 90;
int position_servo3 = 120;
int krok_serva = 2;
int motor_power = 80;
bool L_G_light = false; // pro blikani zelene LED - indikuje, ze deska funguje
int8_t axis[7] = {5,6,7,8,9,10,11};
byte btn[8] = {0,0,0,0,0,0,0,0};
byte btn_last[8] = {0,0,0,0,0,0,0,0};

BluetoothSerial SerialBT;
Stream* serial = nullptr;

timeout send_data { msec(500) }; // timeout zajistuje posilani dat do PC kazdych 500 ms

void setup() {
    Serial.begin(115200);
    if (!SerialBT.begin("RoadAssistance")) //Bluetooth device name
    {
        Serial.println("!!! Bluetooth initialization failed!");
        serial = &Serial;
    }
    else
    {
        serial = &SerialBT;
        SerialBT.println("!!! Bluetooth work!");
        Serial.println("!!! Bluetooth work!");
    }
    // odriveSerial.begin(115200, SERIAL_8N1, 13, 15);

    // delay(2000);
    // Serial.println("Starting");
    // odrive.run_state(0, ODriveArduino::AXIS_STATE_FULL_CALIBRATION_SEQUENCE, true);
    // if (!odrive.run_state(1, ODriveArduino::AXIS_STATE_FULL_CALIBRATION_SEQUENCE, true))
    //     Serial.println("Calibration sequence failed");
    // odrive.run_state(0, ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL, true);
    // if (!odrive.run_state(1, ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL, true))
    //     Serial.println("Failed to set closed loop reulation");

    // Serial.println("Calibration done");
    // delay(2000);

    // Serial.println("Going up");
    // for (int i = 0; i != 10000; i += 1) {
    //     odrive.SetPosition(0, i);
    //     odrive.SetPosition(1, i);
    //     delay(1);
    // }
    // Serial.println("Going down");
    // for (int i = 10000; i != 0; i -= 1) {
    //     odrive.SetPosition(0, i);
    //     odrive.SetPosition(1, i);
    //     delay(1);
    // }
    // Serial.println("Stop");
    servo0.attach(27);
    servo0.write(position_servo0);
    servo1.attach(26);
    servo1.write(position_servo1);
    servo2.attach(4);
    servo2.write(position_servo2);
    servo3.attach(32);
    servo3.write(position_servo3);
    delay(500);
    servo3.write(position_servo3-5);
    send_data.restart();
}

void testovaci(); // dole pod main 
void read_joystick();

void loop() {
    if (send_data) {
        send_data.ack();
        if (L_G_light) L_G_light = false; else  L_G_light = true;
        rbc().leds().green(L_G_light);
        // Serial.println( millis() );
        // SerialBT.println( millis() ); // na tomto pocitaci COM port 13
    }

    // testovaci();
    read_joystick();
    if ( (btn[4]==1) and (btn_last[4]==0) )
        rbc().setMotors().power(OTOCNY_MOTOR, -motor_power)
                         .set();
    if ( (btn[4]==0) and (btn_last[4]==1) )
        rbc().setMotors().power(OTOCNY_MOTOR, 0)
                         .set();
    
    if ( (btn[5]==1) and (btn_last[5]==0) )
        rbc().setMotors().power(OTOCNY_MOTOR, motor_power)
                         .set();

    if ( (btn[5]==0) and (btn_last[5]==1) )
        rbc().setMotors().power(OTOCNY_MOTOR, 0)
                         .set();
 
    
    


    

    


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

// ********************************************************************

void read_joystick(){
if (SerialBT.available() > 0)
    {
        uint8_t test = SerialBT.read();
        if (test == 0x80)
            for (uint8_t x = 0; x < 7; x++)
            {
			    while(SerialBT.available() < 1) {
				    // DO NOTHING - WAITING FOR PACKET
				    delay(1);
			    }

                axis[x] = SerialBT.read();
                Serial.print(x);
                Serial.print(": ");
                Serial.print(axis[x], DEC);
                Serial.print(" ");
            }
        else if  ( test == 0x81 )
        {
		    while(SerialBT.available() < 1) {
			    // DO NOTHING - WAITING FOR PACKET
			    delay(1);
		    }
            byte a = SerialBT.read();
		    while(SerialBT.available() < 1) {
			    // DO NOTHING - WAITING FOR PACKET
			    delay(1);
		    }
            btn_last[a] = btn[a]; 
            btn[a] = SerialBT.read();
		    Serial.print(a, DEC); Serial.print(": "); Serial.print(btn[a], DEC); Serial.print("last: "); Serial.print(btn_last[a], DEC);
        }
        Serial.println(" ");
        
    }
}


// ********************************************************************
void testovaci()
{
   char c;
    if(Serial.available() or SerialBT.available() )  {
        if(Serial.available()) c = Serial.read(); else c = SerialBT.read(); 
        // Serial.write("c: "); Serial.println(c); Serial.write(" "); Serial.println(c,DEC);
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
            case 'b':
                rbc().setMotors().power(OTOCNY_MOTOR, motor_power)
                                .set();
                break;    
            case 'm':
                rbc().setMotors().power(OTOCNY_MOTOR, -motor_power)
                                .set();
                break; 

            case ' ':
                rbc().setMotors().power(OTOCNY_MOTOR, 0)
                                .set();
                break; 

            default:
                Serial.write(c);
                break;
        }
    }

}