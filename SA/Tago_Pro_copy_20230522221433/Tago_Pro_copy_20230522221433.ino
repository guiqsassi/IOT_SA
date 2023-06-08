//exemplo base
//referência: https://raw.githubusercontent.com/vinicius0082/esp32-tagoIO/main/exemplos/mpu_esp32_mqtt_tagoIO


//Envio de Dados para tagoIO via MQTT

#include <ArduinoJson.h>
#include "EspMQTTClient.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>


//pinos de entrada e saída
Adafruit_MPU6050 mpu;

//declaração das variáveis do sensor mpu6050
const uint8_t scl = 32;
const uint8_t sda = 33;

//declaração do buzzer
const int buzzer = 26;

//declaração dos leds
const int ledBranco = 27;
const int ledAmarelo = 14;
const int ledVermelho = 12;

//variáveis para Json
char json_acelX[100];
char json_acelY[100];
char json_acelZ[100];
char json_temperatura[100];
// char json_giroX[100];
// char json_giroY[100];
// char json_giroZ[100];

//variáveis internas
const uint8_t MPU6050SlaveAddress = 0x68;



const uint16_t AccelScaleFactor = 16384;

const uint16_t GyroScaleFactor = 131;


const uint8_t MPU6050_REGISTER_SMPLRT_DIV =  0x19;

const uint8_t MPU6050_REGISTER_USER_CTRL =  0x6A;

const uint8_t MPU6050_REGISTER_PWR_MGMT_1 =  0x6B;

const uint8_t MPU6050_REGISTER_PWR_MGMT_2 =  0x6C;

const uint8_t MPU6050_REGISTER_CONFIG =  0x1A;

const uint8_t MPU6050_REGISTER_GYRO_CONFIG =  0x1B;

const uint8_t MPU6050_REGISTER_ACCEL_CONFIG =  0x1C;

const uint8_t MPU6050_REGISTER_FIFO_EN =  0x23;

const uint8_t MPU6050_REGISTER_INT_ENABLE =  0x38;

const uint8_t MPU6050_REGISTER_ACCEL_XOUT_H =  0x3B;

const uint8_t MPU6050_REGISTER_SIGNAL_PATH_RESET = 0x68;

int16_t AccelX, AccelY, AccelZ, Temperature, GyroX, GyroY, GyroZ;

//gap para ter um intervalo entre os sons do buzzer
int gap=1000;

const int SAMPLING_FREQ = 100;


//configurações da conexão MQTT
EspMQTTClient client
(
  "FIESC_IOT", //nome da sua rede Wi-Fi
  "C6qnM4ag81", //senha da sua rede Wi-Fi
  "mqtt.tago.io",  // MQTT Broker server ip padrão da tago
  "Token",   // username
  "b3ed8e3e-f9e7-4945-8eae-a475f3a5bc17",   // Código do Token
  "SafeVibration",     // Client name that uniquely identify your device
  1883              // The MQTT port, default to 1883. this line can be omitted
);

//conectar o ESP32 a internet
void I2C_Write(uint8_t deviceAddress, uint8_t regAddress, uint8_t data){

  Wire.beginTransmission(deviceAddress);

  Wire.write(regAddress);

  Wire.write(data);

  Wire.endTransmission();

}

//configuração dos pinos
void setup()
{
  Serial.begin(115200);
  
  Wire.begin(sda, scl);

  MPU6050_Init();

  pinMode(27, OUTPUT);

  pinMode(14,OUTPUT);

  pinMode(12,OUTPUT);

  pinMode(buzzer, OUTPUT);

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

//loop do programa
void loop()
{
  leitura_sinais();
  converte_json();
  envia_msg();

  delay(1000);

  client.loop();
}

//inicializa o mpu6050, são protocolos de comunicação entre a ESP32 e o mpu6050
void MPU6050_Init(){

  delay(150);

  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SMPLRT_DIV, 0x07);

  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_1, 0x01);

  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_2, 0x00);

  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_CONFIG, 0x00);

  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_GYRO_CONFIG, 0x00);

  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_CONFIG, 0x00);

  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_FIFO_EN, 0x00);

  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_INT_ENABLE, 0x01);

  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SIGNAL_PATH_RESET, 0x00);

  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_USER_CTRL, 0x00);

}

//lendo os valores do mpu6050
void Read_RawValue(uint8_t deviceAddress, uint8_t regAddress){

  Wire.beginTransmission(deviceAddress);

  Wire.write(regAddress);

  Wire.endTransmission();

  Wire.requestFrom(deviceAddress, (uint8_t)14);

  AccelX = (((int16_t)Wire.read()<<8) | Wire.read());

  AccelY = (((int16_t)Wire.read()<<8) | Wire.read());

  AccelZ = (((int16_t)Wire.read()<<8) | Wire.read());

  Temperature = (((int16_t)Wire.read()<<8) | Wire.read());

  // GyroX = (((int16_t)Wire.read()<<8) | Wire.read());

  // GyroY = (((int16_t)Wire.read()<<8) | Wire.read());

  // GyroZ = (((int16_t)Wire.read()<<8) | Wire.read());

}

//tratando os dados do mpu6050 e mostrando eles no serial
void leitura_sinais()
{
  double Ax, Ay, Az, T, Gx, Gy, Gz;

 
  
  Read_RawValue(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_XOUT_H);
 

  Ax =(double)AccelX/AccelScaleFactor  ;

  Ay = (double)AccelY/AccelScaleFactor ;

  Az = (double)AccelZ/AccelScaleFactor ;

  T = (double)Temperature/340+36.53;

  // Gx = (double)GyroX/GyroScaleFactor;

  // Gy = (double)GyroY/GyroScaleFactor;

  // Gz = (double)GyroZ/GyroScaleFactor;

  delay(1000 / SAMPLING_FREQ);
 

  Serial.print("Ax: "); Serial.print(Ax);

  Serial.print(" Ay: "); Serial.print(Ay);

  Serial.print(" Az: "); Serial.print(Az);

  Serial.print(" T: "); Serial.println(T);

  // Serial.print(" Gx: "); Serial.print(Gx);

  // Serial.print(" Gy: "); Serial.print(Gy);

  // Serial.print(" Gz: "); Serial.println(Gz);

}

// funções zap1, zap2, risefall, fall, rise, twotone, são referentes ao som do buzzer

//--------------------------------------------------------------
void zap1()
{
    for (float f=3000;f>40;f=f*0.93){
    tone(buzzer,f);
    delay(10);
  }
}

void zap2()
{
    for (float f=3000;f>10;f=f*0.85){
    tone(buzzer,2*f);
    delay(5);
    tone(buzzer,f);
    delay(5); 
  }
}
void risefall()
{
  float rise_fall_time=180;
  int steps=50;
  float f_max=2600;
  float f_min=1000;
  float delay_time=rise_fall_time/steps;
  float step_size=(f_max-f_min)/steps;
  for (float f =f_min;f<f_max;f+=step_size){
    tone(buzzer,f);
    delay(delay_time);
  }
   for (float f =f_max;f>f_min;f-=step_size){
    tone(buzzer,f);
    delay(delay_time);
  }
}
void fall(float rise_fall_time)
{
  int steps=50;
  float f_max=2000;
  float f_min=500;
  float delay_time=rise_fall_time/steps;
  float step_size=0.97;
  for (float f =f_max;f>f_min;f*=step_size){
    tone(buzzer,f);
    delay(delay_time);
  }
}
void rise()
{
  float rise_fall_time=2000;
  int steps=100;
  float f_max=1500;
  float f_min=500;
  float delay_time=rise_fall_time/steps;
  float step_size=1.012;
  for (float f =f_min;f<f_max;f*=step_size){
    tone(buzzer,f);
    delay(delay_time);
  }
  noTone(buzzer);
  delay(100);
  
}

void twotone()
{
  float f_max=1500;
  float f_min=1000;
  float delay_time=800;
  tone(buzzer,f_max);
  delay(delay_time);
  tone(buzzer,f_min);
  delay(delay_time);
  
}
//--------------------------------------------------------------

// converte os dados de leitura_sinais para JSON
void converte_json()
{
  StaticJsonDocument<300> sjson_acelX;
  StaticJsonDocument<300> sjson_acelY;
  StaticJsonDocument<300> sjson_acelZ;
  StaticJsonDocument<300> sjson_temperatura;
  // StaticJsonDocument<300> sjson_giroX;
  // StaticJsonDocument<300> sjson_giroY;
  // StaticJsonDocument<300> sjson_giroZ;
  
  sjson_acelX["variable"] = "acelX";
  sjson_acelX["value"] = (double)AccelX/AccelScaleFactor;
  serializeJson(sjson_acelX, json_acelX);
  
  sjson_acelY["variable"] = "acelY";
  sjson_acelY["value"] = (double)AccelY/AccelScaleFactor;
  serializeJson(sjson_acelY, json_acelY);
  
  
  sjson_acelZ["variable"] = "acelZ";
  sjson_acelZ["value"] = (double)AccelZ/AccelScaleFactor;
  serializeJson(sjson_acelZ, json_acelZ);
  
  sjson_temperatura["variable"] = "temperatura";
  sjson_temperatura["value"] = Temperature/340+36.53;
  serializeJson(sjson_temperatura, json_temperatura);
  
  // sjson_giroX["variable"] = "giroX";
  // sjson_giroX["value"] = GyroX;
  // serializeJson(sjson_giroX, json_giroX);
  
  // sjson_giroY["variable"] = "giroY";
  // sjson_giroY["value"] = GyroY;
  // serializeJson(sjson_giroY, json_giroY);
  
  // sjson_giroZ["variable"] = "giroZ";
  // sjson_giroZ["value"] = GyroZ;
  // serializeJson(sjson_giroZ, json_giroZ);
  
}

//enviando os dados JSON do mpu6050 para o TAGO.IO
void envia_msg()
{
  client.publish("node/acelerometro", json_acelX);
  client.publish("node/acelerometro", json_acelY);
  client.publish("node/acelerometro", json_acelZ);
  client.publish("node/acelerometro", json_temperatura);
  // client.publish("node/acelerometro", json_giroX);
  // client.publish("node/acelerometro", json_giroY);
  // client.publish("node/acelerometro", json_giroZ);
}

//conexão do ESP32 com o tópico actuators do TAGO.IO
void onConnectionEstablished()
{
   client.subscribe("node/actuators", [] (const String &payload)  {
   Serial.println(payload);
   processa_msg(payload);
  });
}

//traz os dados do TAGO.IO e faz as condições para ligar os leds e o buzzer
void processa_msg(const String payload)
{
  StaticJsonDocument<300> msg;

  DeserializationError err = deserializeJson(msg, payload);
  if (err) {
    Serial.print(F("deserializeJson() failed with code "));
    Serial.println(err.f_str());
  }
  Serial.print("var:");  
  String var = msg["variable"];
  Serial.println(var);
  
  
    if(var == "acely")
  {
    Serial.print("value:");
    String val = msg["value"];
    float valVib = val.toFloat();
    Serial.println(valVib);

    if(valVib >= -400 || valVib <= 40){
          digitalWrite(ledBranco, LOW);
      digitalWrite(ledVermelho, LOW);
      digitalWrite(ledAmarelo, HIGH);
    }
    if(valVib >= 50){
      digitalWrite(ledVermelho, HIGH);
         digitalWrite(buzzer, HIGH);
         digitalWrite(ledAmarelo, LOW);
         digitalWrite(ledBranco, LOW);
  //      for (int count=1;count<=10;count++)
  // {
  //   risefall();
  // }
  // noTone(buzzer);
  // delay(gap);
  // for (int count=1;count<=10;count++)
  // {
  //   fall(300);
  // } 
  // noTone(buzzer);
  // delay(gap); 
  // for (int count=1;count<=5;count++)
  // {
  //   fall(600);
  // }
  // noTone(buzzer);
  // delay(gap); 
  // for (int count=1;count<5;count++)
  // {
  //   rise();
  // }
  // noTone(buzzer);
  // delay(gap); 
  // for (int count=1;count<5;count++)
  // {
  //   twotone();
  // }
  // noTone(buzzer);
  // delay(gap); 
  // for (int count=1;count<10;count++)
  // {
  //   zap1();
  // }
  // noTone(buzzer);
  // delay(gap); 
  for (int count=1;count<10;count++)
  {
    zap2();
  }
  noTone(buzzer);
  digitalWrite(ledBranco, LOW);
  digitalWrite(ledAmarelo, LOW);
  delay(gap);  
            
      } 

    else{
    digitalWrite(ledBranco, HIGH);
    digitalWrite(ledVermelho, LOW);
    digitalWrite(ledAmarelo, LOW);
  }
  }

}





