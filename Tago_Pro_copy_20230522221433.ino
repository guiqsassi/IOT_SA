//exemplo base
//referência: https://raw.githubusercontent.com/vinicius0082/esp32-tagoIO/main/exemplos/mpu_esp32_mqtt_tagoIO


//Envio de Dados para tagoIO via MQTT

#include <ArduinoJson.h>
#include "EspMQTTClient.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>


//pinos de entrada e saída

const uint8_t scl = 32;
const uint8_t sda = 33;

const int buzzer = 26;

const int ledBranco = 27;
const int ledAmarelo = 14;
const int ledVermelho = 12;

//variáveis para Json
char json_acelX[100];
char json_acelY[100];
char json_acelZ[100];
char json_temperatura[100];
char json_giroX[100];
char json_giroY[100];
char json_giroZ[100];

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

int gap=1000;




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

  

}
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

void I2C_Write(uint8_t deviceAddress, uint8_t regAddress, uint8_t data){

  Wire.beginTransmission(deviceAddress);

  Wire.write(regAddress);

  Wire.write(data);

  Wire.endTransmission();

}

void Read_RawValue(uint8_t deviceAddress, uint8_t regAddress){

  Wire.beginTransmission(deviceAddress);

  Wire.write(regAddress);

  Wire.endTransmission();

  Wire.requestFrom(deviceAddress, (uint8_t)14);

  AccelX = (((int16_t)Wire.read()<<8) | Wire.read());

  AccelY = (((int16_t)Wire.read()<<8) | Wire.read());

  AccelZ = (((int16_t)Wire.read()<<8) | Wire.read());

  Temperature = (((int16_t)Wire.read()<<8) | Wire.read());

  GyroX = (((int16_t)Wire.read()<<8) | Wire.read());

  GyroY = (((int16_t)Wire.read()<<8) | Wire.read());

  GyroZ = (((int16_t)Wire.read()<<8) | Wire.read());

}

void MPU6050_Init(){

  delay(150);

  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SMPLRT_DIV, 0x07);

  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_1, 0x01);

  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_2, 0x00);

  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_CONFIG, 0x00);

I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_GYRO_CONFIG, 0x00);//set +/-250 degree/second full scale

  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_CONFIG, 0x00);// set +/- 2g full scale

  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_FIFO_EN, 0x00);

  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_INT_ENABLE, 0x01);

  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SIGNAL_PATH_RESET, 0x00);

  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_USER_CTRL, 0x00);

}

void leitura_sinais()
{
  double Ax, Ay, Az, T, Gx, Gy, Gz;

 
  
  Read_RawValue(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_XOUT_H);
  delay(1000);
 

  Ax = (double)AccelX/AccelScaleFactor;

  Ay = (double)AccelY/AccelScaleFactor;

  Az = (double)AccelZ/AccelScaleFactor;

  T = (double)Temperature/340+36.53;

  Gx = (double)GyroX/GyroScaleFactor;

  Gy = (double)GyroY/GyroScaleFactor;

  Gz = (double)GyroZ/GyroScaleFactor;

 

  Serial.print("Ax: "); Serial.print(Ax);

  Serial.print(" Ay: "); Serial.print(Ay);

  Serial.print(" Az: "); Serial.print(Az);

Serial.print(" T: "); Serial.print(T);

  Serial.print(" Gx: "); Serial.print(Gx);

  Serial.print(" Gy: "); Serial.print(Gy);

  Serial.print(" Gz: "); Serial.println(Gz);

}


void converte_json()
{
  StaticJsonDocument<300> sjson_acelX;
  StaticJsonDocument<300> sjson_acelY;
  StaticJsonDocument<300> sjson_acelZ;
  StaticJsonDocument<300> sjson_temperatura;
  StaticJsonDocument<300> sjson_giroX;
  StaticJsonDocument<300> sjson_giroY;
  StaticJsonDocument<300> sjson_giroZ;

  // sjson_acelerometro["variable"] = "acelerometro";
  // sjson_acelerometro["value"] = AccelX, AccelY, AccelZ, Temperature, GyroX, GyroY, GyroZ;
  // serializeJson(sjson_acelerometro, json_acelerometro);
  
  sjson_acelX["variable"] = "acelX";
  sjson_acelX["value"] = AccelX;
  serializeJson(sjson_acelX, json_acelX);
  
  sjson_acelY["variable"] = "acelY";
  sjson_acelY["value"] = AccelY;
  serializeJson(sjson_acelY, json_acelY);
  
  
  sjson_acelZ["variable"] = "acelZ";
  sjson_acelZ["value"] = AccelZ;
  serializeJson(sjson_acelZ, json_acelZ);
  
  sjson_temperatura["variable"] = "temperatura";
  sjson_temperatura["value"] = Temperature/340+36.53;
  serializeJson(sjson_temperatura, json_temperatura);
  
  sjson_giroX["variable"] = "giroX";
  sjson_giroX["value"] = GyroX;
  serializeJson(sjson_giroX, json_giroX);
  
  sjson_giroY["variable"] = "giroY";
  sjson_giroY["value"] = GyroY;
  serializeJson(sjson_giroY, json_giroY);
  
  sjson_giroZ["variable"] = "giroZ";
  sjson_giroZ["value"] = GyroZ;
  serializeJson(sjson_giroZ, json_giroZ);
  
}

void envia_msg()
{
  client.publish("node/acelerometro", json_acelX); // You can activate the retain flag by setting the third parameter to true
  client.publish("node/acelerometro", json_acelY);
  client.publish("node/acelerometro", json_acelZ);
  client.publish("node/acelerometro", json_temperatura);
  client.publish("node/acelerometro", json_giroX);
  client.publish("node/acelerometro", json_giroY);
  client.publish("node/acelerometro", json_giroZ);
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
  
    if(var == "led_branco")
  {
    Serial.print("value:");
    String val = msg["value"];
    Serial.println(val);
    if(val == "activate")
      digitalWrite(ledBranco, LOW);
    else
      digitalWrite(ledBranco, HIGH);
  }
    if(var == "temperatura")
  {
    Serial.print("value:");
    String val = msg["value"];
    Serial.println(val);
    float valTemp = val.toFloat();
    Serial.println("valTemp");

    if(valTemp >= 27){
      digitalWrite(ledVermelho, HIGH);
         digitalWrite(buzzer, HIGH);
       for (int count=1;count<=10;count++)
  {
    risefall();
  }
  noTone(buzzer);
  delay(gap);
  for (int count=1;count<=10;count++)
  {
    fall(300);
  } 
  noTone(buzzer);
  delay(gap); 
  for (int count=1;count<=5;count++)
  {
    fall(600);
  }
  noTone(buzzer);
  delay(gap); 
  for (int count=1;count<5;count++)
  {
    rise();
  }
  noTone(buzzer);
  delay(gap); 
  for (int count=1;count<5;count++)
  {
    twotone();
  }
  noTone(buzzer);
  delay(gap); 
  for (int count=1;count<10;count++)
  {
    zap1();
  }
  noTone(buzzer);
  delay(gap); 
  for (int count=1;count<10;count++)
  {
    zap2();
  }
  noTone(buzzer);
  delay(gap);  
            
      } 

    else{
      digitalWrite(ledVermelho, LOW);
      digitalWrite(buzzer, LOW);
      tone(buzzer, 0);
  }
  }
    if(var == "led_amarelo")
  {
    Serial.print("value:");
    String val = msg["value"];
    Serial.println(val);
    if(val == "activate")
      digitalWrite(ledAmarelo, LOW);
    else
      digitalWrite(ledAmarelo, HIGH);
  }
}

// This function is called once everything is connected (Wifi and MQTT)
// WARNING : YOU MUST IMPLEMENT IT IF YOU USE EspMQTTClient
void onConnectionEstablished()
{
   client.subscribe("node/actuators", [] (const String &payload)  {
   Serial.println(payload);
   processa_msg(payload);
  });
}


