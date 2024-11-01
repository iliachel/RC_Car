#include <esp_now.h>
#include <WiFi.h>
#include <GyverButton.h>

#define axisXLpin 32 //pin for axisX on joystick
#define axisYLpin 34 //pin for axisY on joystick
#define axisZLpin 14 //pin for axisZ (button) on joystick
#define axisXRpin 36 //pin for axisX on joystick
#define axisYRpin 39 //pin for axisY on joystick
#define axisZRpin 27 //pin for axisZ (button) on joystick
#define axisGainpin 35

unsigned long timing;
uint8_t broadcastAddress[] = {0xEC, 0xDA, 0x3B, 0xAA, 0xB1, 0x7C}; //dest. mac address

GButton axisZL(axisZLpin);
GButton axisZR(axisZRpin);


int L_X_Joystick_Value = 2000;
int L_Y_Joystick_Value = 2000;
int R_X_Joystick_Value = 1700;
int R_Y_Joystick_Value = 1700;
int Gain_Value = 2000;

typedef struct data
{
  int axisXL;
  int axisZL;
  int axisYR;
  int axisZR;
  int axisGain;

} data; //struct for sending data via ESP-NOW

data sendingData; //object to send values via ESP-NOW

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) 
{
  //Serial.print("\r\nLast Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() 
{
  Serial.begin(115200);

  pinMode(axisXLpin, INPUT);
  pinMode(axisYRpin, INPUT);
  pinMode(axisGainpin, INPUT);
  WiFi.mode(WIFI_STA);

   if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
 
  esp_now_register_send_cb(OnDataSent); //configuring ESP to sender status
  
  esp_now_peer_info_t peerInfo {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}



void loop() 
{
  if(millis() - timing <=20) 
  {
    return;
  }
  timing = millis();

  L_X_Joystick_Value = analogRead(axisXLpin);
  L_X_Joystick_Value = analogRead(axisXLpin);
  
  R_Y_Joystick_Value = analogRead(axisYRpin);
  R_Y_Joystick_Value = analogRead(axisYRpin);

  Gain_Value = analogRead(axisGainpin);
  Gain_Value = analogRead(axisGainpin);
  
  sendingData.axisXL = L_X_Joystick_Value;
  sendingData.axisYR = R_Y_Joystick_Value;
  sendingData.axisGain = Gain_Value;
  sendingData.axisZL = 0;
  sendingData.axisZR = 0;


  axisZL.tick();
  axisZR.tick();

  if (axisZL.isClick())
  {
    sendingData.axisZL = 1;
  }
  if (axisZR.isClick())
  {
    sendingData.axisZR = 1;
  }

  //Serial.println(sendingData.axisXL);
  Serial.println(sendingData.axisGain);
  //Serial.println(sendingData.axisZL);
  //Serial.println(sendingData.axisZR);
  //Serial.println(analogRead(axisXLpin));
  //Serial.println(analogRead(axisYLpin));
  //Serial.println(analogRead(axisXRpin));
  //Serial.println(analogRead(axisYRpin));

  esp_now_send(broadcastAddress, (uint8_t *) &sendingData, sizeof(sendingData)); //send data
}
