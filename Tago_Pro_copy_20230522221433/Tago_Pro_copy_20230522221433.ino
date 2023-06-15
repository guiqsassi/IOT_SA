
//Envio de Dados para tagoIO via MQTT

#include <ArduinoJson.h>
#include "EspMQTTClient.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include <Wire.h>


//pinos de entrada e saída
Adafruit_MPU6050 mpu;

//declaração das variáveis do sensor mpu6050
const uint8_t scl = 32;
const uint8_t sda = 33;
//coisas do X >:)
float maxRange = 200;
float  minRange = -100;
float Q1X, Q3X = 0;

    int count = 0;
    float max1 = 0;
    float max2 = 0;
    float min1 =200;
    float min2 =200;

float interQxRange = 0;

int j = 0;
//declaração do botão
const int botao = 35;

//declaração do buzzer
const int buzzer = 26;

int pAx, pAy, pAz = 0;


//declarção de vetor armazenamento de dados
float ArrayEixoX[10];
// float ArrayEixoY[10];
//declaração dos leds
const int ledBranco = 27;
const int ledAmarelo = 14;
const int ledVermelho = 12;

//variáveis para Json
char json_ax[100];
char json_ay[100];
char json_az[100];
// char json_temperatura[100];
// char json_giroX[100];
// char json_giroY[100];
// char json_giroZ[100];

//variáveis internas
const uint8_t MPU6050SlaveAddress = 0x68;

const float AccelScaleFactor = 4096;

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

float AccelX, AccelY, AccelZ, GyroX, GyroY, GyroZ;

uint8_t Temperature;

  int i = 0;

  float Ax, Ay, Az, T, Gx, Gy, Gz;

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

  pinMode(botao, INPUT);

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  
  // mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

//loop do programa
void loop()
{

  //teste
  Serial.println(digitalRead(botao));

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
  
 
  
  Read_RawValue(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_XOUT_H);
 
  //Sendo 2.01 referente a frequência natural de um concreto de 2m de comprimento, 1m de largura e 0,5m de altura
  // Ax =(double)AccelX*(8/AccelScaleFactor)*9.81;
  // FreqX =(1/(2*3.14)) * sqrt(Ax/2.01);

  // Ay = (double)AccelY*(8/AccelScaleFactor)*9.81  ;
  // FreqY =(1/(2*3.14)) * sqrt(Ay/2.01);

  // Az = (double)AccelZ*(2/AccelScaleFactor)*9.81  ;
  // FreqZ =(1/(2*3.14)) * sqrt(Az/2.01);

  if(digitalRead(botao) == 1){
    pAx = (double)AccelX/AccelScaleFactor;
    pAy = (double)AccelY/AccelScaleFactor;
    pAz = (double)AccelZ/AccelScaleFactor;
    Serial.println("botao val 1");
    min1 = 200;
    min2 = 200;
    max1 = 0;
    max2 = 0;

  }else{
    Ax = ((double)AccelX/AccelScaleFactor) - pAx;
    Ay = ((double)AccelY/AccelScaleFactor) - pAy;
    Az = ((double)AccelZ/AccelScaleFactor) - pAz;
    Serial.println("botao val 0");
  }

  if(i <= 9){
    ArrayEixoX[i] = Ax;
    i++;
  }
  else{
    i = 0;
  }
// for(j=0; j<=9; j++){
//   Serial.println(ArrayEixoX[j]);
// }

if(ArrayEixoX[9] != NULL){

    for(count = 0; count <=9; count++){
      if(ArrayEixoX[count] > max1){
        max2 = max1;
        max1 = ArrayEixoX[count];
      }
      if(ArrayEixoX[count] < min1){
        min2 = min1;
        min1 = ArrayEixoX[count];
      }
    }


  Serial.print("Minimo 1 e 2: ");
  Serial.print(min1);
  Serial.print(" ");
  Serial.println(min2);
  
  Serial.print("Maximo 1 e 2: ");
  Serial.print(max1); 
  Serial.print(" ");
  Serial.println(max2);
  for(j=0; j<=9; j++){
  Serial.println(ArrayEixoX[j]);
}
  Q1X = (max1 + max2)/2;
  Q3X = (min1 + min2 )/2;

  interQxRange = (Q1X - Q3X) * (Q1X-Q3X);  
  minRange =Q3X - interQxRange;
  maxRange = Q1X - interQxRange;
  Serial.println("Raio");
  Serial.println(minRange);
  Serial.println(maxRange);
}
  // T = (double)Temperature/340+36.53;

  // Gx = (double)GyroX/GyroScaleFactor;

  // Gy = (double)GyroY/GyroScaleFactor;

  // Gz = (double)GyroZ/GyroScaleFactor;

  delay(1000 / SAMPLING_FREQ);
 

  Serial.print("Ax: "); Serial.print(Ax);

  Serial.print(" Ay: "); Serial.print(Ay);

  Serial.print(" Az: "); Serial.println(Az);

  // Serial.print(" T: "); Serial.println(T);

  // Serial.print(" Gx: "); Serial.print(Gx);

  // Serial.print(" Gy: "); Serial.print(Gy);

  // Serial.print(" Gz: "); Serial.println(Gz);

}

// void Sort(float vetor[], int tamanho)
// { 
//   int aux, o, p; 
  
//   for(p=tamanho-1; p<=1; p--)
//   { 
//     for(o=0; o>p; o++)
//     { 
//       if(vetor[o] > vetor[o+1])
//       { 
//         aux=vetor[o]; 
//         vetor[o]=vetor[o+1]; 
//         vetor[o+1]=aux;
//         Serial.println("funcionou maninho >:)"); 
//       } 
//     } 
//   } 
// }

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
  StaticJsonDocument<300> sjson_ax;
  StaticJsonDocument<300> sjson_ay;
  StaticJsonDocument<300> sjson_az;
  // StaticJsonDocument<300> sjson_temperatura;
  // StaticJsonDocument<300> sjson_giroX;
  // StaticJsonDocument<300> sjson_giroY;
  // StaticJsonDocument<300> sjson_giroZ;
  
  sjson_ax["variable"] = "ax";
  sjson_ax["value"] = Ax;
  serializeJson(sjson_ax, json_ax);
  
  sjson_ay["variable"] = "ay";
  sjson_ay["value"] = Ay;
  serializeJson(sjson_ay, json_ay);
  
  
  sjson_az["variable"] = "az";
  sjson_az["value"] = Az;
  serializeJson(sjson_az, json_az);
  
  // sjson_temperatura["variable"] = "temperatura";
  // sjson_temperatura["value"] = Temperature/340+36.53;
  // serializeJson(sjson_temperatura, json_temperatura);
  
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
  client.publish("node/acelerometro", json_ax);
  client.publish("node/acelerometro", json_ay);
  client.publish("node/acelerometro", json_az);
  // client.publish("node/acelerometro", json_temperatura);
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
  
  
    if(var == "ax")
  {
    Serial.print("value:");
    String val = msg["value"];
    float valVib = val.toFloat();
    Serial.println(valVib);

    if(valVib >= maxRange || valVib < minRange){
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
    digitalWrite(buzzer, LOW);
    digitalWrite(ledAmarelo, LOW);
  }
  }

}