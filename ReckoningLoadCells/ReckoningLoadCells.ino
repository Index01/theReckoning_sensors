#include <Ethernet.h>
#include <EthernetUdp.h>
#include <SPI.h>    
#include <OSCMessage.h>
#include "HX711.h"


// Pin defs
const uint8_t pin_cal_btn = 5;
const uint8_t pin_OSC_led = 1;
const uint8_t pin_trim = A5;
const uint8_t pin_led_ext = A1;
// HX711 pins 
const uint8_t PIN_CLK_LOADCELL1 = 6; 
const uint8_t PIN_DATA_LOADCELL1 = 9;  
const uint8_t PIN_CLK_LOADCELL2 = 11;
const uint8_t PIN_DATA_LOADCELL2 = 12; 
// Load cells
const uint8_t loadCellsMaxLbs = 120;
const long LOADCELL_OFFSET = 50682624;
const long LOADCELL_DIVIDER = 5895655;
HX711 loadCell1;
HX711 loadCell2;
// Ethernetz
EthernetUDP Udp;
// Arduino's IP
IPAddress ip(128, 32, 122, 252);
// OSC destination IP
IPAddress outIp(128, 32, 122, 125);
const unsigned int outPort = 9999;
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED}; 


void calibrateLoadCells(){ 
  Serial.println("[+] Calibrating ...");
  loadCell1.set_scale(LOADCELL_DIVIDER);
  loadCell1.set_offset(LOADCELL_OFFSET);

  loadCell2.set_scale(LOADCELL_DIVIDER);
  loadCell2.set_offset(LOADCELL_OFFSET);

  loadCell1.tare();
  loadCell2.tare();
  Serial.println("[+] Scale, Offset, Tare complete");
}


int32_t readLoadCell(int lcNum){
  int32_t reading = 0;
  switch(lcNum){
    case 1:
      reading = loadCell1.get_value(5);
      Serial.print("[+] LC1 reading(lbs): ");
      Serial.println(reading);
      break;
    case 2:
      reading = loadCell2.get_value(5);
      Serial.print("[+] LC2 reading(lbs): ");
      Serial.println(reading);
      break;
    default:
      Serial.print("[-] No viable load cells");
      break;
  }
  return reading;
}


void sendOSCMsgs(int32_t lc1, int32_t lc2){
  //the message wants an OSC address as first argument
  OSCMessage msg1("/LoadCell1");
  msg1.add(lc1);
  OSCMessage msg2("/LoadCell2");
  msg2.add(lc2);

  Udp.beginPacket(outIp, outPort);
  msg1.send(Udp); // send the bytes to the SLIP stream
  msg2.send(Udp);
  Udp.endPacket(); // mark the end of the OSC Packet
  msg1.empty(); // free space occupied by message
  msg2.empty(); 

  Serial.println("[+] OSC msg sent");
  analogWrite(pin_OSC_led, 1);
  delay(200);
  analogWrite(pin_OSC_led, 0);
}


void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("[+] Welcome Starfighter");
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(pin_cal_btn, INPUT_PULLUP);
  pinMode(pin_led_ext, OUTPUT);
   
  // Initialize the HX711
  loadCell1.begin(PIN_DATA_LOADCELL1, PIN_CLK_LOADCELL1);
  loadCell2.begin(PIN_DATA_LOADCELL2, PIN_CLK_LOADCELL2);
  calibrateLoadCells();
  

  //Ethernet.begin(mac,ip);
  //delay(20);

  //Udp.begin(8888);
  //delay(20);
}


// Ooh goodie, more globals
const int task1_interval = 1000;
const int task2_interval = 1000;
const int task3_interval = 2000;
const int task4_interval = 2000;
int task1_nextExecution = 0;
int task2_nextExecution = 0;
int task3_nextExecution = 0;
int task4_nextExecution = 0;
int calStateLast = LOW;
bool ledState = 0;
int trimMinWeight = 0;


void loop(){
  int now = millis();
  /** Heartbeat **/
  if(now>task1_nextExecution){
    task1_nextExecution = now+task1_interval;
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState);
  }
  /** Read the load cells and send the OSC msgs. **/
  if(now>task2_nextExecution){
    Serial.println("[+] Executing readLoadCell and sendOSCMsgs");
    task2_nextExecution = now+task2_interval;
    int32_t lc1 = readLoadCell(1);
    int32_t lc2 = readLoadCell(2);
    if(lc1>=trimMinWeight && lc2>=trimMinWeight){
        // sendOSCMsgs(lc1, lc2);
        digitalWrite(pin_led_ext, HIGH);
        delay(600);
        digitalWrite(pin_led_ext, LOW);
    }
  }
  /** Check if we should re-calibrate. **/
  if(now>task3_nextExecution){
    task3_nextExecution = now+task3_interval;
    int calStateCurrent = digitalRead(pin_cal_btn);
    if(calStateCurrent==LOW && calStateLast==LOW){
      Serial.println("[+] Calibrate button pressed");
      calibrateLoadCells();
      for(int i=0; i<5; i++){
        digitalWrite(LED_BUILTIN, HIGH);
        digitalWrite(pin_led_ext, HIGH);
        delay(150);
        digitalWrite(LED_BUILTIN, LOW);
        digitalWrite(pin_led_ext, LOW);
        delay(150);
      }
    }
    calStateLast = calStateCurrent;
  }
  /** Trim pot **/ 
  if(now>task4_nextExecution){
    task4_nextExecution = now+task4_interval;
    float trimVal = analogRead(pin_trim);
    float trimPercent = trimVal/4095;
    int trimMin = loadCellsMaxLbs * trimPercent;
    if(trimMin<=1){
      trimMinWeight = -1000;
    }
    else{
      trimMinWeight = trimMin;
    }
    Serial.print("[+] Min trim weight: ");
    Serial.println(trimMinWeight);
  }
}

