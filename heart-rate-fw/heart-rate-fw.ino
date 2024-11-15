/*!
 * @file  gainHeartbeatSPO2.ino
 * @n experiment phenomena: get the heart rate and blood oxygenation, during the update the data obtained does not change
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author      PengKaixing(kaixing.peng@dfrobot.com)
 * @version     V1.0.0
 * @date        2021-06-21
 * @url         https://github.com/DFRobot/DFRobot_BloodOxygen_S
 */
#include "DFRobot_BloodOxygen_S.h"

#define UART_COMMUNICATION  //use I2C for communication, but use the serial port for communication if the line of codes were masked

#ifdef  I2C_COMMUNICATION
#define I2C_ADDRESS    0x57
  DFRobot_BloodOxygen_S_I2C MAX30102(&Wire ,I2C_ADDRESS);
#else
/* ---------------------------------------------------------------------------------------------------------------
 *    board   |             MCU                | Leonardo/Mega2560/M0 |    UNO    | ESP8266 | ESP32 |  microbit  |
 *     VCC    |            3.3V/5V             |        VCC           |    VCC    |   VCC   |  VCC  |     X      |
 *     GND    |              GND               |        GND           |    GND    |   GND   |  GND  |     X      |
 *     RX     |              TX                |     Serial1 TX1      |     5     |   5/D6  |  D2   |     X      |
 *     TX     |              RX                |     Serial1 RX1      |     4     |   4/D7  |  D3   |     X      |
 * ---------------------------------------------------------------------------------------------------------------*/
#if defined(ARDUINO_AVR_UNO) || defined(ESP8266)
SoftwareSerial mySerial(4, 5);
DFRobot_BloodOxygen_S_SoftWareUart MAX30102(&mySerial, 9600);
#else
DFRobot_BloodOxygen_S_HardWareUart MAX30102(&Serial1, 9600); 
#endif
#endif

const int ledPin =  12;      // the number of the LED pin

void setup()
{
  Serial.begin(115200);
  
  initsensor();

  pinMode(ledPin, OUTPUT);
}

void initsensor(){
while (false == MAX30102.begin())
  {
    Serial.println("init fail!");
    delay(1000);
  }
  Serial.println("init success!");
  Serial.println("start measuring...");
  MAX30102.sensorStartCollect();
}

unsigned long prevMil_readSensor = 0;
int intervalSensor = 1000;



// Variables will change:
int ledState = LOW;             // ledState used to set the LED
long previousMillis = 0;        // will store last time LED was updated
// the follow variables is a long because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long interval = 100;           // interval at which to blink (milliseconds)
long interval2 = 50;  
long interval3 = 50;  

int hrate = 60;
int mappedDelay;
int status = 0;
void loop()
{


readSensor();
  
  
if(status == 1){
digitalWrite(ledPin, HIGH);
  delay(100);
  digitalWrite(ledPin, LOW);
  delay(50);
  digitalWrite(ledPin, HIGH);
  delay(100);
  digitalWrite(ledPin, LOW);

  delay(mappedDelay);
} else {
doTheFade(millis());
}
  

  // if (ledState == LOW) // if the led is OFF then check if its time to blink it 
  // { 
  //      unsigned long currentMillis = millis();
  //   if(currentMillis - previousMillis > interval) {
  //     // save the last time you blinked the LED 
  //     previousMillis = currentMillis;   
  //     ledState = HIGH;
  //     // set the LED with the ledState of the variable:
  //     digitalWrite(ledPin, ledState);
  //   }
  // }
  // else // if the led is ON wait just a shorter period of time (interval2) and turn it OFF
  // {
  //    unsigned long currentMillis = millis();
  //   if(currentMillis - previousMillis > interval2) {
  //     // save the last time you blinked the LED 
  //     previousMillis = currentMillis;   
  //       ledState = LOW;
  //     // set the LED with the ledState of the variable:
  //     digitalWrite(ledPin, ledState);
  //   }
  // }
}

void readSensor(){
//if(millis() - prevMil_readSensor > intervalSensor){
    prevMil_readSensor = millis();

    MAX30102.getHeartbeatSPO2();
  Serial.print("SPO2 is : ");
  Serial.print(MAX30102._sHeartbeatSPO2.SPO2);
  Serial.println("%");
  Serial.print("heart rate is : ");
  Serial.print(MAX30102._sHeartbeatSPO2.Heartbeat);
  if(MAX30102._sHeartbeatSPO2.Heartbeat > 0) {
    hrate = MAX30102._sHeartbeatSPO2.Heartbeat;
    status = 1;
  } else {
    status = 0;
  }
   Serial.print(" / ");
  Serial.print(hrate);
  Serial.println(" Times/min");
  Serial.print("Temperature value of the board is : ");
  Serial.print(MAX30102.getTemperature_C());
  Serial.println(" ℃");

  mappedDelay = map(hrate, 50, 160, 1000, 100);

  Serial.print("mappedDelay: ");
  Serial.print(mappedDelay);
  Serial.println("");

 // }
}


const byte pwmLED = 5;

// define directions for LED fade
#define UP 0
#define DOWN 1

// constants for min and max PWM
const int minPWM = 0;
const int maxPWM = 255;

// State Variable for Fade Direction
byte fadeDirection = UP;

// Global Fade Value
// but be bigger than byte and signed, for rollover
int fadeValue = 0;

// How smooth to fade?
byte fadeIncrement = 5;

// millis() timing Variable, just for fading
unsigned long previousFadeMillis;

// How fast to increment?
int fadeInterval = 50;

int led = 9;         // the PWM pin the LED is attached to
int brightness = 0;  // how bright the LED is
int fadeAmount = 5;  // how many points to fade the LED by


void doTheFade(unsigned long thisMillis) {

   // set the brightness of pin 9:
  analogWrite(ledPin, brightness);

  // change the brightness for next time through the loop:
  brightness = brightness + fadeAmount;

  // reverse the direction of the fading at the ends of the fade:
  if (brightness <= 0 || brightness >= 255) {
    fadeAmount = -fadeAmount;
  }
  // wait for 30 milliseconds to see the dimming effect
  delay(15);
   // is it time to update yet?
   // if not, nothing happens
  //  if (thisMillis - previousFadeMillis >= fadeInterval) {
  //     // yup, it's time!
  //     if (fadeDirection == UP) {
  //        fadeValue = fadeValue + fadeIncrement;
  //        if (fadeValue >= maxPWM) {
  //           // At max, limit and change direction
  //           fadeValue = maxPWM;
  //           fadeDirection = DOWN;
  //        }
  //     } else {
  //        //if we aren't going up, we're going down
  //        fadeValue = fadeValue - fadeIncrement;
  //        if (fadeValue <= minPWM) {
  //           // At min, limit and change direction
  //           fadeValue = minPWM;
  //           fadeDirection = UP;
  //        }
  //     }
  //     // Only need to update when it changes
  //     analogWrite(pwmLED, fadeValue);

  //     // reset millis for the next iteration (fade timer only)
  //     previousFadeMillis = thisMillis;
  //  }
}
