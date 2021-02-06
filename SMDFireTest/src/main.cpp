#include <M5Stack.h>
#include "sm_protocol.h"
sm_protocol servo(500000);
int error = 0;
unsigned int counter = 0;
float velocity = 50;

uint8_t byte_data = 0;
uint16_t short_data = 0;
uint16_t target_pwm = 0;
uint16_t encoder_count = 0;
uint16_t current_speed = 0;

void setup()
{

  M5.begin();
  M5.Power.begin();
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(GREEN, BLACK);
  M5.Lcd.setTextSize(2);


    // set PWM control mode
  byte_data = REG_CONTROL_MODE_PWM;
  error = servo.writeByteCommand(1, REG_CONTROL_MODE, 1, &byte_data);
  if (error != 0)
  {
    M5.Lcd.setCursor(0, 30);
    M5.Lcd.printf("control mode error : %i", error);
  }


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
  short_data = 0;
  error = servo.writeWordCommand(1, REG_GOAL_PWM_100_L, 1, &target_pwm);

  error = servo.readWordCommand(1, REG_ENCODER_ABSOLUTE_L, 1, &encoder_count);

  error = servo.readWordCommand(1, REG_PRESENT_VELOCITY_DPS_L, 1, &current_speed);
    


    
  M5.Lcd.setCursor(0, 20);
  M5.Lcd.printf("PWM target : %i            ", target_pwm);
  M5.Lcd.setCursor(0, 40);
  M5.Lcd.printf("encoder count : %i         ", encoder_count);
  M5.Lcd.setCursor(0, 60);
  M5.Lcd.printf("current dps  : %i          ", current_speed);
}
