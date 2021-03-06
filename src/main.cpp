/*
  main.cpp
  Main code where the control takes place
  @author  Leo Korbee (c), Leo.Korbee@xs4all.nl
  @website iot-lab.org
  @license Attribution-NonCommercial-ShareAlike 4.0 International (CC BY-NC-SA 4.0)
  Thanks to all the folks who contributed beforme me on this code.

  Hardware information at the end of this file.

  @version 2020-11-22
  Succesfull with MiniPill LoRa (STM32L051 processor) after changing code from
  ATTiny84. There is no downlink window! Advantage is the lower power consumption,
  but it is not following the LoRaWAN specifications.

  @version 2020-11-23
  Add pseudo random number function to switch between frequencies in RFM95.cpp

  @version 2020-11-27
  Added ISP.end in main code to disable ISP system. It saves about 66uA and
  reduced the total power usage to 1.3uA without BME280

  @version 2020-11-28
  Added BME280, total power usage 1.5uA in sleep mode

*/
#include <Arduino.h>
#include "BME280.h"
#include "LoRaWAN.h"
#include "STM32IntRef.h"
#include "STM32LowPower.h"
#include "secconfig.h" // remember to rename secconfig_example.h to secconfig.h and to modify this file

/*
  for debugging purposes, usualy not enough memory to use this with both BME
  and RFM95W
*/
// for debugging redirect to hardware Serial2
// Tx on PA2
//#define Serial Serial2
//HardwareSerial Serial2(USART2);   // or HardWareSerial Serial2 (PA3, PA2);

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

// Sleep this many microseconds.
#define SLEEP_INTERVAL 300000

// all functions declared
void disableIO();

void setup()
{
  // for debugging
  // Serial.begin(9600);

  // disableIO();

  //Initialize RFM module
  rfm.init();
  lora.setKeys(NwkSkey, AppSkey, DevAddr);
  delay(500);

  // begin communication with BME280 and set to default sampling, iirc, and standby settings
  if (bme.begin() < 0)
  {
     // Serial.println("Error communicating with BME280 sensor, please check wiring");
     while(1){}
  }

  // Configure low power at startup
  LowPower.begin();

  // use this delay for first packet send 8 seconds after reset
  delay(8000);
}

void loop()
{
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

  bme.goToSleep();
  // set PA6 to analog to reduce power due to currect flow on DIO on BME280
  // reduces 50uA
  pinMode(PA6, INPUT_ANALOG);
  // disable SPI, reduces 67uA
  SPI.endTransaction();
  SPI.end();

  // go to low power mode
  LowPower.begin();
  // take SLEEP_INTERVAL time to sleep
  LowPower.deepSleep(SLEEP_INTERVAL);
  // start SPI after sleep
  SPI.begin();
}


/*
  set unused pins so no undefined situation takes place
*/
void disableIO(void)
{
  GPIO_InitTypeDef GpioA;
  uint32_t gpio_pins = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
  GpioA.Pin = gpio_pins;
  GpioA.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GpioA);

  // No Reduction due to not using GpioB
  GPIO_InitTypeDef GpioB;
  gpio_pins = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | \
                      GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | \
                      GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | \
                      GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  GpioB.Pin = gpio_pins;
  GpioB.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GpioB);

  __HAL_RCC_GPIOB_CLK_DISABLE();
  __HAL_RCC_GPIOC_CLK_DISABLE();
  __HAL_RCC_GPIOH_CLK_DISABLE();
}

/*
  Hardware setup
  MiniPill LoRa (iot-lab.org)
  STM32L051C8T6

  P2 connector
  	PA_0,  // Analog in
  	PA_1,  // Analog in                 CSB - BME280
  	PA_2,  // USART2_TX
  	PA_3,  // USART2_RX
  	VDD
  	GND
  	PA_4,  // SPI1_NSS   NSS - RFM95W
  	PA_5,  // SPI1_SCK.  SCK - RFM95W   SCL - BME280
  	PA_6,  // SPI1_MISO. MISO - RFM95W  SDO - BME280
  	PA_7,  // SPI1_MOSI. MOSI - RFM95W  SDA - BME280
  	VDD
  	GND

  P3 connector
  	PA_9,  // USART1_TX. RST - RFM95W
  	PA_10, // USART1_RX. DIO0 - RFM95W
  	PB_4,  //            DIO1 - RFM95W
  	PB_5,  //            DIO2 - RFM95W
  	PB_6,  // USART1_TX
  	PB_7,  // USART1_RX
  	PB_8,  // I2C1_SCL
  	PB_9,  // I2C1_SDA
  	PB_10, // LPUART1_TX
  	PB_11, // LPUART1_RX
  	VDD
  	GND


*/
