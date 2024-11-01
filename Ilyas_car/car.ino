#include <pwmWrite.h>
#include <WiFi.h>
#include <esp_now.h>
//#include "mpu.h" // MPU-6050 handling (.h file in sketch folder)


#define steering_Servo_Pin 5 //Steering Servo
#define forward_Movement_Pin 3 //Forward movement pin on you're main motor driver
#define backward_Movement_Pin 2 //Backward movement pin on you're main motor driver

Pwm pwm = Pwm();

unsigned long timing; //Variable, necessary to set the delay of execution of a command

const int default_Steering_Servo_Position = 85; //Default servo angle for steering
const int max_Steering_Servo_Position = 105; //Max angle for steering servo
const int min_Steering_Servo_Position = 65; //Min angle for steering servo

const int default_Forward_Movement_Speed = 0;
const int max_Forward_Movement_Speed = 255; //Max speed for motor forward movement (0-255)
const int min_Forward_Movement_Speed = 40; //Min speed for motor forward movement (do not change)

const int default_Backward_Movement_Speed = 0;
const int max_Backward_Movement_Speed = 150; //Max speed for motor backward movement (0-255)
const int min_Backward_Movement_Speed = 40; //Min speed for motor backward movement (do not change)

int forward_Movement_Speed = min_Forward_Movement_Speed; //Current Forward Movement Speed. Default value: minimum
int backward_Movement_Speed = min_Backward_Movement_Speed; //Current Backward Movement Speed. Default value: minimum
int steering_Servo_Position = default_Steering_Servo_Position; //Current steering servo position
int reset_To_Default = 0; //Variable to reset car status
int led_Status = 0; //Variable to enable drift mode
int break_Status = 0;
int back_Led_Status = 0;

typedef struct data
{
  int l_X_Axis;
  int l_Z_Axis;
  int r_Y_Axis;
  int r_Z_Axis;
  int Gain_Axis;

} data; //struct for received data via ESP-NOW

data received_Data; //object to get values from ESP-NOW

void OnDataReceive(const uint8_t * mac, const uint8_t * incoming_Data, int lenght) 
{
  memcpy(&received_Data, incoming_Data, sizeof(received_Data)); //Copying data from buffer to our object
}

void setup() 
{
  delay(3000); //let esp wake-up
  Serial.begin(115200);
  pinMode(forward_Movement_Pin, OUTPUT); //Pin configuration to control motor
  pinMode(backward_Movement_Pin, OUTPUT); //Pin configuration to control motor
  pinMode(steering_Servo_Pin, OUTPUT); //Pin configuration for steering servo
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);

  received_Data.l_X_Axis = 2000; //Setting default values for received data. This will allow the machine to remain in place
  received_Data.r_Y_Axis = 1700; //until it receives another command from the remote control

  WiFi.mode(WIFI_STA); //WiFi configuration for ESP-NOW

  if(esp_now_init() != 0) //Check ESP-NOW initialization
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataReceive); //Register our ESP to receive data;
}




void loop() 
{
  if(millis() - timing <= 20) //Timeout for ESP to prevent unnecessary actions
  {
    return;
  }
  timing = millis();

  reset_To_Default = received_Data.l_Z_Axis;
  break_Status = 0;
  if(received_Data.r_Z_Axis == 1) //check backward speed is zero
  {
    if(led_Status == 0)
    {
      led_Status = 1;
    }
    else
    {
      led_Status = 0;
    }
  }

  int joystick_R_Y_Val = received_Data.r_Y_Axis;

  if(joystick_R_Y_Val < 1600)
  {
    steering_Servo_Position=map(joystick_R_Y_Val, 0, 1600, min_Steering_Servo_Position,  default_Steering_Servo_Position);
  }
  else if(joystick_R_Y_Val > 1800 )
  {
    steering_Servo_Position=map(joystick_R_Y_Val,  4100, 1800, max_Steering_Servo_Position, default_Steering_Servo_Position);
  }
  //Serial.println(received_Data.l_X_Axis);



  int joystick_L_X_Val = received_Data.l_X_Axis;
  
  if(joystick_L_X_Val > 2500)
  {
    forward_Movement_Speed = default_Forward_Movement_Speed;
    backward_Movement_Speed = map(joystick_L_X_Val, 3000, 4100, min_Backward_Movement_Speed, max_Backward_Movement_Speed); 
    back_Led_Status = 1;
  }
  else if(joystick_L_X_Val < 1600 )
  {
    backward_Movement_Speed = default_Backward_Movement_Speed;
    forward_Movement_Speed = map(joystick_L_X_Val, 1000, 0, min_Forward_Movement_Speed, max_Forward_Movement_Speed);
    
  } else {
    backward_Movement_Speed = default_Backward_Movement_Speed;
    forward_Movement_Speed = default_Forward_Movement_Speed;
    break_Status = 1;
    back_Led_Status = 0;
  }



  if(reset_To_Default == 1)
  {
    forward_Movement_Speed = min_Forward_Movement_Speed;
    backward_Movement_Speed = min_Backward_Movement_Speed;
    steering_Servo_Position = default_Steering_Servo_Position;
    led_Status = 0;
  }

  if(!break_Status)
  

  if(led_Status == 1)
  {
    analogWrite(9, 50);
    digitalWrite(8, HIGH);
  } else
  {
    analogWrite(9, 0);
    digitalWrite(8, LOW);
  }

  if(break_Status == 1)
  {
    analogWrite(9, 255);
  } else 
  {
    analogWrite(9, 0);
  }

  if(back_Led_Status == 1)
  {
    analogWrite(7, 255);
  } else
  {
    analogWrite(7, 0);
  }

  pwm.writeServo(steering_Servo_Pin, steering_Servo_Position);
  analogWrite(forward_Movement_Pin, forward_Movement_Speed);
  analogWrite(backward_Movement_Pin, backward_Movement_Speed);

  //Serial.println(back_Led_Status);
}
