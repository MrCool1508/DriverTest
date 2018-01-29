//#include <I2Cdev.h>

#include "T3_softI2C.h"
#include "myI2Cdev.h"

SoftI2C mySoftI2C;
//myI2Cdev  myHardwareI2Cdev;
//I2Cdev    HardwareI2Cdev;

I2C_HandleTypeDef hi2c1;
#define BufferSize 0x100
uint8_t WriteBuffer[BufferSize], ReadBuffer[BufferSize];

#define kLEFT_BRAKE_PIN      BDPIN_GPIO_10 //PE13
#define kLEFT_DIRECTION_PIN  BDPIN_GPIO_8  //PE11
#define kRIGHT_BRAKE_PIN     BDPIN_GPIO_11 //PE14
#define kRIGHT_DIRECTION_PIN BDPIN_GPIO_9  //PE12

#define kLEFT_FORWARD        digitalWrite(kLEFT_DIRECTION_PIN, LOW)
#define kLEFT_BACK           digitalWrite(kLEFT_DIRECTION_PIN, HIGH)
#define kLEFT_RELEASE_BRAKE  digitalWrite(kLEFT_BRAKE_PIN, LOW)
#define kLEFT_BRAKE          digitalWrite(kLEFT_BRAKE_PIN, HIGH)

#define kRIGHT_FORWARD        digitalWrite(kRIGHT_DIRECTION_PIN, HIGH)
#define kRIGHT_BACK           digitalWrite(kRIGHT_DIRECTION_PIN, LOW)
#define kRIGHT_RELEASE_BRAKE  digitalWrite(kRIGHT_BRAKE_PIN, LOW)
#define kRIGHT_BRAKE          digitalWrite(kRIGHT_BRAKE_PIN, HIGH)

void setup() {
  // put your setup code here, to run once:
  pinMode(BDPIN_LED_USER_1, OUTPUT);
//  myHardwareI2Cdev.begin(100);
//  HardwareI2Cdev.begin(100);
  Serial.begin(115200); 
  pinMode(BDPIN_GPIO_4, OUTPUT);
  digitalWrite(BDPIN_GPIO_4, HIGH);
  pinMode(BDPIN_GPIO_5, OUTPUT);
  digitalWrite(BDPIN_GPIO_5, HIGH);
  pinMode(BDPIN_GPIO_6, OUTPUT);
  digitalWrite(BDPIN_GPIO_6, HIGH);
  pinMode(BDPIN_GPIO_7, OUTPUT);
  digitalWrite(BDPIN_GPIO_7, HIGH);

//  /*Direction & Brake*/
  pinMode(BDPIN_GPIO_8, OUTPUT);
  pinMode(BDPIN_GPIO_9, OUTPUT);
  pinMode(BDPIN_GPIO_10, OUTPUT);
  pinMode(BDPIN_GPIO_11, OUTPUT);
      kLEFT_FORWARD;        
 //  kLEFT_BACK    ;       
   kLEFT_RELEASE_BRAKE; 
//    kLEFT_BRAKE         ;

    kRIGHT_FORWARD       ; 
//    kRIGHT_BACK           ;
    kRIGHT_RELEASE_BRAKE ;
//   kRIGHT_BRAKE         ;


};                                                                                                                                        

void loop() {
  // put your main code here, to run repeatedly:
//  uint8_t data[4] = {0};
//  uint8_t testData[2] = {3,4};
///********电机缓慢加速再缓慢减速**********/

  uint8_t data[2];
  uint16_t output;//3072;
  output = 1500;
  uint8_t idx = 0;
  for(idx = 0; idx < 64; idx++){
      output = 1500 + idx * 64;
      if(output > 3000) output = 3000;
      Serial.println(output);
      data[0] = (output/16);
      data[1] = (output%16)<<4;
      mySoftI2C.writeBuffer(kI2C_DAC_LEFT, kI2C_DAC_SLAVE_ADDR, kI2C_DAC_WRITE_ADDR, 2, data);  
      mySoftI2C.writeBuffer(kI2C_DAC_RIGHT, kI2C_DAC_SLAVE_ADDR, kI2C_DAC_WRITE_ADDR, 2, data);  
//      delay_us(10);
      if(output > 4030) delay_s(3);
      delay_s(3);
  }

  Serial.println(output);
  for(idx = 64; idx > 0; idx--){
      output = idx * 64;
      if(output > 4095) output = 4095;
      data[0] = (output/16);
      data[1] = (output%16)<<4;
 //     mySoftI2C.writeBuffer(kI2C_DAC_LEFT, kI2C_DAC_SLAVE_ADDR, kI2C_DAC_WRITE_ADDR, 2, data);  
 //     mySoftI2C.writeBuffer(kI2C_DAC_RIGHT, kI2C_DAC_SLAVE_ADDR, kI2C_DAC_WRITE_ADDR, 2, data);  
      delay_ms(50);
  }
///********电机缓慢加速再缓慢减速**********/

//   uint8_t data[4] ;//= {0};
//   Serial.println(data[0]);
//   Serial.println(data[1]);
//   Serial.println(data[2]);
//   Serial.println(data[3]);
//   uint8_t idx;
//   uint8_t flag;
//  
//   flag = myHardwareI2Cdev.readBit(0x08, 4, data);
//   flag = HardwareI2Cdev.readBit(0x08, 0, 4, 0);
//   Serial.println(flag); 
//   Serial.println(data[0]);
//   Serial.println(data[1]);
//   Serial.println(data[2]);
//   Serial.println(data[3]);
//  
//   flag = mySoftI2C.readBuffer(kI2C_SENSOR, 0x70, 0, 6, data);
//   Serial.println(flag); 
//   while(flag){
//      for(idx = 6; idx > 0; idx--)
//      {
//          Serial.println(data[idx]); 
//      }
//   }

//for(idx = 0; idx < 64; idx++){
//      output = idx * 64;
//      if(output > 4095) output = 4095;
//      data[0] = (output/16);
//      data[1] = (output%16)<<4;
//      Serial.println(data[0]);
//      Serial.println(data[1]);
//      flag = myHardwareI2Cdev.readBytes(kI2C_REVERSE0_SLAVE_ADDR, 2, data);
//      delay_ms(50);
//      if(output > 4030) delay_s(3);
//  }

 // myHardwareI2Cdev.writeBytes(0xA0, 0x02, 2, testData);  
  //myHardwareI2Cdev.readBytes(0xA0, 0x02, 2, data);
//  flag = myHardwareI2Cdev.masterRead(0x70, 4, data);
//  if(HAL_I2C_Master_Receive(&hi2c1, 0x71, ReadBuffer, 4, 0x05) == HAL_OK)
//      Serial.println("\r\n Master Receive OK! \r\n"); // slave:infrared(0x70), because of receive((0x70 << 1)+1);
//  else
//      Serial.println("\r\n Master Receive Failed! \r\n");
//  for(idx = 0; idx < 4 ; idx++){
//      Serial.println(ReadBuffer[idx]);
//  }
//
//  Serial.println(flag);
  Serial.println("***********\r\n");
//  
//  Serial.println(data[0]);
//  Serial.println(data[1]);
//  Serial.println(data[2]);
//  Serial.println(data[3]);
  
//  delay_s(1);  
  digitalWrite(BDPIN_LED_USER_1, HIGH);  // set to as HIGH LED is turn-off
  delay_ms(500);                   // Wait for 0.1 second

  digitalWrite(BDPIN_LED_USER_1, LOW);   // set to as LOW LED is turn-on
  delay_ms(500);                   // Wait for 0.1 second
  Serial.println("end!"); 
}
