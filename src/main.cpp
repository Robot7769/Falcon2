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
#define PNR 57.29577951308232
#define SERVOMAX 180
#define SERVOMIN 0
#define AXIS_COUNT 8
#define BTN_COUNT 11


using rb::LED_GREEN;
void configureOdrive( ODriveArduino& odrive );

#define OTOCNY_MOTOR  rb::MotorId::M1 // motor pro otaceni servoruky
HardwareSerial odriveSerial(1);
ODriveArduino odrive(odriveSerial);

rb::Manager& rbc()
{
    static rb::Manager m(false,false);  // ve výchozím stavu se motory po puštění tlačítka vypínají, false zařídí, že pojedou, dokud nedostanou další pokyn
    return m;
}

Servo servo0, servo1, servo2, servo3;
int position_servo0 = 85;
int position_servo1 = 165;
int position_servo2 = 180;
int position_servo3 = 160;
float krok_arm = 1.20;
int krok_serva = 3;
int motor_power = 99;
bool L_G_light = false; // pro blikani zelene LED - indikuje, ze deska funguje
int otocka_kola = 13 * 2400 ; // převodovka (1:5) 1:8,  2400 tiků enkodéru na otáčku motoru
long max_speed = 20000; // pocet tiku za sekundu max cca 200000,  enkodéry zvládají cca 5000 otacek motoru za sekundu
int speed_coef = 200; // nasobeni hodnoty, co leze z joysticku

int axis[AXIS_COUNT] = {5,6,7,8,9,10,11};
byte btn[BTN_COUNT] = {0,0,0,0,0,0,0,0};
byte btn_changed[BTN_COUNT] = {0,0,0,0,0,0,0,0};

BluetoothSerial SerialBT;
Stream* serial = nullptr;

timeout send_data { msec(1000) }; // timeout zajistuje posilani dat do PC kazdych 500 ms

void setup() {
    Serial.begin(115200);
    if (!SerialBT.begin("RoadAssistance")) //Bluetooth device name // na tomto pocitaci COM port 13
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
    Serial.print ("RBC initialized\n");
    // auto& batt = rbc().battery();
    // batt.setCoef(9.0);
    servo0.attach(27);
    servo0.write(position_servo0);
    servo1.attach(26);
    servo1.write(position_servo1);
    servo2.attach(4);
    servo2.write(position_servo2);
    servo3.attach(32);
    servo3.write(position_servo3);


    rbc().leds().blue( true );  // zapne modrou LED - tim zapne i Odrive
    pinMode(GPIO_NUM_21,OUTPUT );
    digitalWrite(GPIO_NUM_21, HIGH);

    odriveSerial.begin(115200, SERIAL_8N1, 13, 15);
    Serial.println( "Setup odrive begin" );

    odrive.initializeMotors( false );  // true - plná kalibrace,  false - kalibrace bez počátečního pískání
    if ( odrive.error() )
    {             // zjistí, jestli je chyba
        odrive.dumpErrors();           // vypíše chybu
        while ( true )
        {
            Serial.println( "Plese fix it!" );
            Serial.print("Voltage: ");
            Serial.println(odrive.inputVoltage() / 4); // vypíše průměrné napětí na článek
            delay( 10000 );
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

        odrive.move( 0, 0, max_speed ); // dojeď s osou 0 na pozici 0 rychlostí max_speed tiků na otáčku
        odrive.move( 1, 0, max_speed );
        delay( 4000 );

        odrive.move( 0, otocka_kola / 2, max_speed );
        odrive.move( 1, otocka_kola / 2, max_speed );
        delay( 1000 );

        odrive.move( 0, 0, max_speed );
        odrive.move( 1, 0, max_speed );
        delay( 1000 );

        if ( odrive.error() )
            odrive.dumpErrors();

    // odrive.turnOff();  // vypíná odrive
    // Serial.println( "Turned off" );

    // while( true ) {
    //     delay( 500 );
    //     Serial.print( "Pos: " );
    //     Serial.println( odrive.getPos( 0 ) );  // vraci pozici enkoderu osy 0
    // }

    // void speed( int axis, float speed ); // pro osu axis nastavi rychlost speed v ticich za sekundu
    // void setAccel( float accel );  // nastavi zrychleni pro dalsi pohyb


    // delay(500);
       send_data.restart();

}
void arm();
void testovaci(); // dole pod main
bool read_joystick(); // dole pod main

void loop() {
    if (send_data)
    {
        send_data.ack();
        if (L_G_light) L_G_light = false; else  L_G_light = true;
        rbc().leds().green(L_G_light);
        //Serial.println( millis() );
        SerialBT.println( millis() ); // na pocitaci Burda COM port 13
    }

    arm();
    //testovaci();
    if ( read_joystick() )
    {
        // float axis_0 = (abs(axis[0]) < 10) ? 0 : axis[0] /128.0;
        // axis_0 = axis_0*axis_0*axis_0;
        // float axis_1 = (abs(axis[1]) < 10) ? 0 : axis[1] /128.0;
        // axis_1 = axis_1*axis_1*axis_1;
        // int levy_m = -(axis_1- (axis_0 /2 )) * speed_coef;
        // int pravy_m = -(axis_1+ (axis_0 /2 )) * speed_coef;
        float axis_0 = (abs(axis[0]) < 15) ? 0 : axis[0];
        float axis_1 = (abs(axis[2]) < 15) ? 0 : axis[2];
        float levy_m = (-axis_1- (axis_0 /2 )) * speed_coef;
        float pravy_m = (-axis_1+ (axis_0 /2 )) * speed_coef;
        odrive.speed( 0 , levy_m );
        odrive.speed( 1 , pravy_m  );
        if ( odrive.error() )
            odrive.dumpErrors();
        printf(" %f %f \n ", levy_m, pravy_m );
        Serial.println(levy_m);
        Serial.println(pravy_m);
        SerialBT.println(levy_m);
        SerialBT.println(pravy_m);
        SerialBT.print(levy_m); SerialBT.print(" "); SerialBT.println(pravy_m);
    }
    delay(10);
}


// ********************************************************************

bool read_joystick()
{
  if ( SerialBT.available() == 0 )
          return false;

      int test = SerialBT.read();
      if (test == 0x80)
      {
          int axis_count = SerialBT.read();
          if (axis_count >= AXIS_COUNT)
          {
              Serial.println("********* CHYBA V POCTU OS !!! ************");
          }
          else
          {
              for (int x = 0; x < axis_count; x++)
              {
                  while(SerialBT.available() < 1)
                  {
                      // DO NOTHING - WAITING FOR PACKET
                      delay(1);
                  }

                  int8_t tmp = SerialBT.read();
                  axis[x] = tmp;
                  Serial.print(x);
                  Serial.print(": ");
                  Serial.print(axis[x], DEC);
                  Serial.print(" ");
                  SerialBT.print(x);
                  SerialBT.print(": ");
                  SerialBT.print(axis[x], DEC);
                  SerialBT.print(" ");

              }
              return true;
          }


      }
      else if  ( test == 0x81 )
      {
          while(SerialBT.available() < 1) {
              // DO NOTHING - WAITING FOR PACKET
              delay(1);
          }
          byte a = SerialBT.read();
          if ( a >= BTN_COUNT )
          {
              Serial.println("********* CHYBA V POCTU TLACITEK !!! ************");
          }
          else
          {
              while(SerialBT.available() < 1) {
                  // DO NOTHING - WAITING FOR PACKET
                  delay(1);
              }
              btn_changed[a] = 1;
              btn[a] = SerialBT.read();
              Serial.print(a, DEC); Serial.print(": "); Serial.print(btn[a], DEC); Serial.print("changed: "); Serial.print(btn_changed[a], DEC);
              return true;
          }

      }
      return false;
}


// ********************************************************************
void testovaci()
{
   char c;
    if(Serial.available() or SerialBT.available() )  {
        if(Serial.available()) c = Serial.read(); else c = SerialBT.read();
        //Serial.write("c: "); Serial.println(c); // Serial.write(" "); Serial.println(c,DEC);
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

        // delay(10);
        // Serial.println(millis());


    }


}

void arm()
{



    if (btn[3]==1) {
      if((position_servo3 < SERVOMAX) and(position_servo3 > SERVOMIN)){
        position_servo2++;
      }
    }
    else if (btn[0]==1) {
      if((position_servo3 < SERVOMAX) and(position_servo3 > SERVOMIN)){
        position_servo2--;
      }
    }

    if (btn[2]==1) {
      if((position_servo3 < SERVOMAX) and (position_servo3 > SERVOMIN)){
        position_servo3--;
      }
    }
    else if (btn[1]==1) {
      if((position_servo3 < SERVOMAX) and (position_servo3 > SERVOMIN)){
        position_servo3++;
      }
    }

    if (axis[6]<-15) {
      position_servo0++;
    }
    else if (axis[6]>15) {
      position_servo0--;
    }

    if (axis[3]<-15) {
      position_servo1++;
    }
    else if (axis[3]>15) {
      position_servo1--;
    }

    if (btn_changed[4]) {
      if (btn[4]) {
        rbc().setMotors().power(OTOCNY_MOTOR, -motor_power)
                         .set();
      } else {
        rbc().setMotors().power(OTOCNY_MOTOR, 0)
                         .set();
      }
      btn_changed[6] = 0;
    }
    if (btn_changed[5]) {
      if (btn[5]) {
        rbc().setMotors().power(OTOCNY_MOTOR, motor_power)
                         .set();
      } else {
        rbc().setMotors().power(OTOCNY_MOTOR, 0)
                         .set();
      }
      btn_changed[5] = 0;
    }

    servo0.write(position_servo0);
    servo1.write(position_servo1);
    servo2.write(position_servo2);
    servo3.write(position_servo3);

}
