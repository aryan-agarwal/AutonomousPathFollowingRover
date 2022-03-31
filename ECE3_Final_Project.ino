#include <ECE3.h>

//VARIABLE DECLARATION
uint16_t sensor_values[8];

const int left_nslp_pin = 31; // nslp ==> awake & ready for PWM
const int left_dir_pin = 29; //left direction pin
const int left_pwm_pin = 40; //left pwm pin

const int right_nslp_pin = 11; // nslp ==> awake & ready for PWM
const int right_dir_pin = 30; //right direction pin
const int right_pwm_pin = 39; //right pwm pin

//SENSOR MINS AND MAXES
const int min[8] = {725, 563, 655, 540, 563, 608, 608, 608 };
const int max[8] = {2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500};
int previous_error = 0;

double kp;
double kd;
int count_all_black = 0;
bool performed_donut = false;

void setup()
{
  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);
  pinMode(right_nslp_pin,OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);

  digitalWrite(left_dir_pin,LOW);
  digitalWrite(left_nslp_pin,HIGH);
  digitalWrite(right_dir_pin,LOW);
  digitalWrite(right_nslp_pin,HIGH);

  ECE3_Init();

  Serial.begin(9600);
  delay(2000);
}

void loop()
{
  bool is_all_black = true;
  int left_speed = 40;
  int right_speed = 40;

  kp = 0.05;
  kd = 0.4;
  
  ECE3_read_IR(sensor_values);

  double fusion_value = 0;
  double callibration_value[8];

  for(int i = 0; i < 8; i++)
  {
    callibration_value[i] = 1000 * (sensor_values[i] - min[i]) / (max[i] - min[i]);
  }
  fusion_value = ((-15 * callibration_value[0]) + (-14 * callibration_value[1]) + (-12 * callibration_value[2]) + (-8 * callibration_value[3]) + (8 * callibration_value[4]) + (12 * callibration_value[5]) + (14 * callibration_value[6]) + (15 * callibration_value[7]))/8;
  //Serial.println(fusion_value);

  double change_in_fusion_value = fusion_value - previous_error;

  if(fusion_value < 0) //turn left
  {
    left_speed -= kp * fusion_value;
  }

  else //turn right
  {
    right_speed += kp *fusion_value;
  }

  if(change_in_fusion_value < 0) //slow down right wheel or speed up left wheel
  {
    right_speed += kd * change_in_fusion_value;
    //left_speed -= kd * change_in_fusion_value;
  }
  else if(change_in_fusion_value > 0)//slow down left wheel or speed up right wheel
  {
    left_speed -= kd * change_in_fusion_value;
    //right_speed += kd * change_in_fusion_value;
  }

  if(fusion_value > -250 && fusion_value < 250) //if the car is on track, increase the speed
  {
    left_speed += 50;
    right_speed += 50;
  }

  for(int i = 0; i < 8; i++)
  {
    if(sensor_values[i] < 2400)
    {
      is_all_black = false;
    }
  }

  if(is_all_black == true)
  {
    count_all_black++;
  }
  else
  {
    count_all_black = 0;
  }

  if(count_all_black == 1)
  {
    if(performed_donut == true)
    {
      analogWrite(left_pwm_pin,0);
      analogWrite(right_pwm_pin,0);
      exit(0);
    }
    performed_donut = true;
    analogWrite(left_pwm_pin,0);
    analogWrite(right_pwm_pin,0);
    delay(10);
    digitalWrite(left_dir_pin,HIGH);
    analogWrite(left_pwm_pin,200);
    analogWrite(right_pwm_pin,200);
    delay(300);
    digitalWrite(left_dir_pin,LOW);
    analogWrite(left_pwm_pin,0);
    analogWrite(right_pwm_pin,0);
    delay(10);
  }
  else
  {
    analogWrite(left_pwm_pin, left_speed);
    analogWrite(right_pwm_pin, right_speed);
  }
  previous_error = fusion_value;
 }
