#include <M5Stack.h>

#include <ros.h>
#include "std_msgs/Int16.h"
#include <WiFi.h>
#include "wifi_credentials.h"

#include "sm_protocol.h"

//ros handle object
ros::NodeHandle nh;
// std_msgs::Int16 Kp;
// std_msgs::Int16 Ki;
// std_msgs::Int16 Kd;
// std_msgs::Int16 Kff;






// Set the rosserial socket server IP address
IPAddress server(192, 168, 0, 30);
// Set the rosserial socket server port
const uint16_t serverPort = 11411;

sm_protocol servo(500000);
int error = 0;
unsigned int counter = 0;
float velocity = 50;

uint8_t byte_data = 0;
uint16_t target_pwm = 0;
uint16_t encoder_count = 0;
uint16_t current_speed = 0;
uint16_t current_voltage = 0;
uint16_t present_current = 0;

uint16_t pid_Kp = 0;
uint16_t pid_Ki = 0;
uint16_t pid_Kd = 0;
uint16_t pid_Kff = 0;


void KpCallback(const std_msgs::Int16 &kp)
{
  nh.loginfo("Kp modification ! ");
  pid_Kp = kp.data;
}



ros::Subscriber<std_msgs::Int16> sub_Kp("Kp", &KpCallback);
// ros::Suscriber<std_msgs::Int16> sub_Ki("Ki", &pid);
// ros::Suscriber<std_msgs::Int16> sub_Kd("Kd", &pid);
// ros::Suscriber<std_msgs::Int16> sub_Kff("Kff", &pid);

void setup()
{

  M5.begin();
  M5.Power.begin();

  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(GREEN, BLACK);
  M5.Lcd.setTextSize(2);

  M5.Lcd.setCursor(0, 0);
  M5.Lcd.print("WiFi...");


  // modify wifi_credentials.h
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    M5.Lcd.print(".");
  }
  M5.Lcd.print("OK !");

  M5.Lcd.setCursor(0, 20);
  M5.Lcd.print("IP address: ");
  M5.Lcd.print(WiFi.localIP());

  nh.getHardware()->setConnection(server, serverPort);

  nh.initNode();

  nh.subscribe(sub_Kp);

  M5.Lcd.setCursor(0, 40);
  M5.Lcd.print("ROSSerial...");

  while (!nh.connected())
  {
    nh.spinOnce();
    delay(100);
  }

  M5.Lcd.print("OK !");

  delay(1000);

  M5.Lcd.fillScreen(BLACK);

  // set PWM control mode
  byte_data = 1;
  error = servo.writeByteCommand(1, REG_TORQUE_ENABLE, 1, &byte_data);
  byte_data = REG_CONTROL_MODE_PWM;
  error = servo.writeByteCommand(1, REG_CONTROL_MODE, 1, &byte_data);
}

void loop()
{

  M5.update();
  if (M5.BtnB.wasPressed())
  {
    target_pwm += 20;
  }

  if (M5.BtnC.wasPressed())
  {
    target_pwm -= 20;
  }

  // set PWM speed
  error = servo.writeWordCommand(1, REG_GOAL_PWM_100_L, 1, &target_pwm);

  error = servo.readWordCommand(1, REG_ENCODER_ABSOLUTE_L, 1, &encoder_count);

  error = servo.readWordCommand(1, REG_PRESENT_VELOCITY_DPS_L, 1, &current_speed);

  error = servo.readWordCommand(1, REG_VOLTAGE_INPUT_ADC_L, 1, &current_voltage);

  error = servo.readWordCommand(1, REG_PRESENT_CURRENT_MA_L, 1, &present_current);

  M5.Lcd.setCursor(0, 20);
  M5.Lcd.printf("target PWM : %i            ", (int16_t)target_pwm);
  M5.Lcd.setCursor(0, 40);
  M5.Lcd.printf("present speed  : %irpm          ", (int16_t)(current_speed)*60 / 360);
  M5.Lcd.setCursor(0, 60);
  M5.Lcd.printf("present current  : %imA         ", (int16_t)(present_current));
  M5.Lcd.setCursor(0, 80);
  M5.Lcd.printf("PID  : KP %i Ki %i Kd %i Kff %i     ", pid_Kp, pid_Ki, pid_Kd, pid_Kff);
  

  M5.Lcd.setCursor(0, 200);
  M5.Lcd.printf("battery voltage  : %i          ", (int16_t)current_voltage);

  delay(100);
}
