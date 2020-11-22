/*
  main.cpp
  Main code where the control takes place
  @author  Leo Korbee (c), Leo.Korbee@xs4all.nl
  @website iot-lab.org
  @license Attribution-NonCommercial-ShareAlike 4.0 International (CC BY-NC-SA 4.0)
  Thanks to all the folks who contributed beforme me on this code.

  Hardware information at the end of this file.

  @version 2020-11-04
  Testing with MiniPill LoRa (STM32L051 processor)


*/
#include <Arduino.h>
//#include <SPI.h>
#include "BME280.h"
#include "LoRaWAN.h"
#include "STM32IntRef.h"
#include "secconfig.h" // remember to rename secconfig_example.h to secconfig.h and to modify this file


/*
  for debugging purposes, usualy not enough memory to use this with both BME
  and RFM95W
*/
// for debugging redirect to hardware Serial2
// Tx on PA2
#define Serial Serial2
HardwareSerial Serial2(USART2);   // or HardWareSerial Serial2 (PA3, PA2);


// RFM95W connection on MiniPill LoRa
#define DIO0 PA10
#define NSS  PA4
RFM95 rfm(SPI, DIO0, NSS);

// define LoRaWAN layer
LoRaWAN lora = LoRaWAN(rfm);
// frame counter for lora
unsigned int Frame_Counter_Tx = 0x0000;

/* A BME280 object using SPI, chip select pin PA1 */
BME280 bme(SPI,PA1);

// the things network stuff
// get them from the device overview page! uncomment and put this information here, remove the #include secconfig.h
// due to security reasons this information from the author is put in secconfig.h

// TTN, msb left
// unsigned char NwkSkey[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
// unsigned char AppSkey[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
// unsigned char DevAddr[4] = { 0x00, 0x00, 0x00, 0x00 };

// sleep cycles that will be counted, start with more than sleep_total to start after 8 seconds with first broadcast.
volatile int sleep_count = 38;

// set sleep time between broadcasts. The processor awakes after 8 seconds deep-sleep_mode,
// increase and checks the counter and sleep again until sleep_total is reached.
// 5min * 60s = 300/8 = 37,5 = 38.
// 17 seconds longer than 5 minutes with 37, so 35 is more apropriate
const int sleep_total = 35; // was 35

// all functions declared
void readData(float &temp, float &hum, float &press);
uint16_t vccVoltage();
void setUnusedPins();
void goToSleep();
void watchdogSetup();


void setup()
{
  // for debugging
  Serial.begin(9600);
  // define unused pins

  Serial.println("Start setup");

  setUnusedPins();

  //Initialize RFM module
  rfm.init();
  lora.setKeys(NwkSkey, AppSkey, DevAddr);

  Serial.println("rfm initialized");

  delay(500);
  Serial.println("starting MiniPill LoRa");

  // begin communication with BME280 and set to default sampling, iirc, and standby settings
  if (bme.begin() < 0)
  {
    Serial.println("Error communicating with BME280 sensor, please check wiring");
    while(1){}
  }

  Serial.println("End setup");

}

void loop()
{

  // goToSleep for all devices...
  // The watchdog timer will reset.
  //goToSleep();
  Serial.println("loop");
  // use this for non-sleep testing:
  delay(8000);
  sleep_count++;

  // do action if sleep_total is reached
  if (sleep_count >= sleep_total)
  {
    Serial.println("sending data");

    // define bytebuffer
    uint8_t Data_Length = 0x09;
	  uint8_t Data[Data_Length];

    // read vcc voltage (mv)
    int32_t vcc = IntRef.readVref();
    Data[0] = (vcc >> 8) & 0xff;
    Data[1] = (vcc & 0xff);

    // reading data from BME sensor
    bme.readSensor();
    float tempFloat = bme.getTemperature_C();
    float humFloat = bme.getHumidity_RH();
    float pressFloat = bme.getPressure_Pa();

    // from float to uint16_t
    uint16_t tempInt = 100 * tempFloat;
    uint16_t humInt = 100 * humFloat;
    // pressure is already given in 100 x mBar = hPa
    uint16_t pressInt = pressFloat/10;

    // move into bytebuffer
    Data[2] = (tempInt >> 8) & 0xff;
    Data[3] = tempInt & 0xff;

    Data[4] = (humInt >> 8) & 0xff;
    Data[5] = humInt & 0xff;

    Data[6] = (pressInt >> 16) & 0xff;
    Data[7] = (pressInt >> 8) & 0xff;
    Data[8] = pressInt & 0xff;

    lora.Send_Data(Data, Data_Length, Frame_Counter_Tx);

    Frame_Counter_Tx++;

    // reset sleep count
    sleep_count = 0;

  }

}


/*
  set unused pins so no undefined situation takes place
*/
void setUnusedPins()
{
  //pinMode(PA0, INPUT_PULLUP);
  //pinMode(PA1, INPUT_PULLUP);
  //pinMode(PA2, INPUT_PULLUP);
  //pinMode(PA3, INPUT_PULLUP);
  //pinMode(PB2, INPUT_PULLUP);
}

/**
 read temperature from BME sensor
*/
void readData(float &temp, float &hum, float &press)
{
  // set units sensor
  // BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
  // BME280::PresUnit presUnit(BME280::PresUnit_Pa);
  // // read sensor (SPI interface)
  // bme.read(press, temp, hum, tempUnit, presUnit);

}

/**
  read voltage of the rail (Vcc)
  output mV (2 bytes)
*/
uint16_t vccVoltage()
{
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  // default ADMUX REFS1 and REFS0 = 0

  // #define _BV(bit) (1 << (bit))

  // 1.1V (I Ref)(2) 100001
  //ADMUX = _BV(MUX5) | _BV(MUX0);

  //delay(2); // Wait for Vref to settle
  //ADCSRA |= _BV(ADSC); // Start conversion
  //while (bit_is_set(ADCSRA,ADSC)); // measuring

  //uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  //uint8_t high = ADCH; // unlocks both

  //uint16_t result = (high<<8) | low;
  uint16_t result = 3300;


  // result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  // number of steps = 1023??
  //result = (1125300L / result) ; // Calculate Vcc (in mV);

  return result;
}

/*
  Hardware setup
  Attiny84 using the Arduino pin numbers! PB0 = 0 etc.


  Atmel ATtiny84A PU
  RFM95W
  BME280

  Power: 3V3 - 470uF elco over power connectors, 100nF over power connector for interference suppression
  Connections:
  RFM95W   ATtiny84

  DIO0 -- PB0
  MISO -- MOSI
  MOSI -- MISO
  SCK  -- SCK
  NSS  -- PB1 (this is Chipselect)

  Bosch BME280 sensor used with tinySPI....
  BME280  ATtiny84
  SCL -- SCK
  SDA -- MISO
  SDO -- MOSI
  CSB -- PA7 (this is Chipselect)
*/
