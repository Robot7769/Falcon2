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
void configureOdrive( ODriveArduino& odrive );

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
int otocka_kola = 5 * 2400 ; // převodovka 1:5,  2400 tiků enkodéru na otáčku motoru
long max_speed = 20000; // pocet tiku za sekundu 
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

    Serial.print ("Starting...\n");
    rbc();
    Serial.print ("RBC initialized1\n");
    auto& batt = rbc().battery();
    batt.setCoef(9.0); 

    odriveSerial.begin(115200, SERIAL_8N1, 13, 15);
    Serial.println( "Setup odrive begin" );
    
    odrive.initializeMotors( false );  // true - plná kalibrace,  false - kalibrace bez počátečního pískání 
    if ( odrive.error() ) {             // zjistí, jestli je chyba
        odrive.dumpErrors();           // vypíše chybu 
        while ( true ) {
            Serial.println( "Plese fix it!" );
            Serial.print("Voltage: ");
            Serial.println(odrive.inputVoltage() / 4); // vypíše průměrné napětí na článek 
            delay( 1000 );
        }
    }
    Serial.println( "Done" );
    Serial.println( "Turning on" );
    odrive.turnOn();  // zapnutí odrive 
    if ( odrive.error() ) {
        odrive.dumpErrors();
    }
    Serial.println( "Turned on" );
    delay( 500 );

    while ( true ) {
          
        odrive.move( 0, 0, max_speed );
        odrive.move( 1, 10 * otocka_kola, max_speed ); // dojeď s osou 1 na pozici ... rychlostí 20000 tiků na otáčku
        delay( 4000 );
        odrive.move( 0, 10 * otocka_kola, max_speed );
        odrive.move( 1, 0, max_speed );
        delay( 4000 );
        if ( odrive.error() )
            odrive.dumpErrors();
    }
    odrive.turnOff();  // vypíná odrive 
    Serial.println( "Turned off" );

    while( true ) {
        delay( 500 );
        Serial.print( "Pos: " );
        Serial.println( odrive.getPos( 0 ) );  // vraci pozici enkoderu osy 0 
    }


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
void read_joystick(); // dole pod main

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
        // delay(10);
        // Serial.println(millis());


    }


}