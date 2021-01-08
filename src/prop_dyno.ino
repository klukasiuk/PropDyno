// Konrad Łukasiuk 2020-2021

#include "HX711.h"
#include <Servo.h>

// Load cells
HX711 thrust(A2, A3);
HX711 torque_A(A4, A5);
HX711 torque_B(A6, A7);

// BLDC motor
Servo motor;

// Consts
const int pwm_arm  = 1000;
const int pwm_low  = 1100;
const int pwm_high = 2000;
const int pwm_step = 25;

const int voltage_pin = A0;
const int current_pin = A1;
const int pwm_pin = 5;

const float load_cell_calib = -412.0f;
const float force_to_torque_ratio = (2 * 9.81 * 0.02685) / 1000.0f;

const float voltage_divider_ratio = 5.13f;
const float current_meter_ratio = -0.066;

const float voltage_alpha = 0.15f;
const float current_alpha = 0.15f;

const int readings = 25;
const int step_readings = 30;
const float inv_readings = 1.0f / readings;

const int uart_baud = 115200;
const int loop_delay = 500;

// Globals
bool armed = false;
bool test = false;

int pwm_value = 0;

float voltage = 0.0;
float current = 0.0;



void check_serial()
{
  if (Serial.available() == 0)
  return;

  String msg = Serial.readString();

  if(msg == "arm\r\n")
  {
    motor.writeMicroseconds(pwm_arm);
    armed = true;
    pwm_value = pwm_arm;
  }

  if(msg == "disarm\r\n")
  {
    motor.writeMicroseconds(0);
    armed = false;
  }

  if(msg == "test\r\n")
  {
    test = true;
    pwm_value = pwm_low;
  }

  if(msg == "tare\r\n")
  {
    thrust.tare();
    moment.tare();
  }

  if(msg == "step\r\n")
  {
    step_response();
  }
}

void step_response()
{
  if(armed == false)
  return;

  float thrust_values[step_readings];
  float moment_values[step_readings];
  int thrust_times[step_readings];
  int moment_times[step_readings];
  
  motor.writeMicroseconds(pwm_low);
  
  delay(500);

  thrust_values[0] = thrust.get_units();
  thrust_times[0] = millis();

  motor.writeMicroseconds(pwm_high);

  // Force readings
  for(int i=1;i<step_readings;i++)
  {
    thrust_values[i] = thrust.get_units();
    thrust_times[i] = millis();
  }

  motor.writeMicroseconds(pwm_low);

  delay(2000);

  torque_values[0] = torque_A.get_units() * force_to_torque_ratio;
  torque_times[0] = millis();

  motor.writeMicroseconds(pwm_high);

  // Torque readings
  for(int i=1;i<step_readings;i++)
  {
    torque_values[i] = torque_A.get_units() * force_to_torque_ratio;
    torque_times[i] = millis();
  }

  motor.writeMicroseconds(pwm_low);
  
  // Print results

  Serial.println("Thrust step response:");
  Serial.print(thrust_times[0]);
  Serial.print(";");
  Serial.print(pwm_low);
  Serial.print(";");
  Serial.println(thrust_values[0], 1);

  for(int j=1;j<step_readings;j++)
  {
    Serial.print(thrust_times[j]);
    Serial.print(";");
    Serial.print(pwm_high);
    Serial.print(";");
    Serial.println(thrust_values[j], 1);
  }

  Serial.println("\nTorque step response:");
  Serial.print(moment_times[0]);
  Serial.print(";");
  Serial.print(pwm_low);
  Serial.print(";");
  Serial.println(moment_values[0], 1);

  for(int j=1;j<step_readings;j++)
  {
    Serial.print(moment_times[j]);
    Serial.print(";");
    Serial.print(pwm_high);
    Serial.print(";");
    Serial.println(moment_values[j], 4);
  }
  
}

float read_voltage_adc()
{
  float voltage_reading = 0.0f;
  
  for(int i=0; i<readings; i++)
  {
    voltage_reading += analogRead(voltage_pin);
  }

  return voltage_reading * (5.0f / 1024) * inv_readings;
}

float read_current_adc()
{
  float current_reading = 0.0f;
  
  for(int i=0; i<readings; i++)
  {
    current_reading += analogRead(current_pin);
  }

  return current_reading  * (5.0f / 1024) * inv_readings;
}

float read_thrust()
{
  float thrust_value = 0.0f;

  for(int i=0; i<readings; i++)
  {
    thrust_value += thrust.get_units();
  }

  thrust_value = thrust_value * inv_readings;

  return thrust_value;
}

float read_torque()
{
  float torque_value = 0.0f;

  for(int i=0; i<readings; i++)
  {
    torque_value += moment.get_units();
  }

  torque_value = torque_value * inv_readings;

  return torque_value;
}

void read_sensors()
{
  if(test == false)
  return;
  
  float thrust_value = read_thrust();
  float torque_value = read_torque();

  float current_reading = read_current_adc();
  float voltage_reading = read_voltage_adc() * voltage_divider_ratio;

  current_reading = (current_reading - 2.5) / current_meter_ratio;
  
  // Low pass filters
  current = current_alpha * current + (1.0 - current_alpha) * current_reading;
  voltage = voltage_alpha * voltage + (1.0 - voltage_alpha) * voltage_reading;
  
  torque_value = torque_value * force_to_torque_ratio;
  
  Serial.print(pwm_value);
  Serial.print(";");
  Serial.print(thrust_value, 1); 
  Serial.print(";");
  Serial.print(torque_value, 4); 
  Serial.print(";");
  Serial.print(voltage, 3); 
  Serial.print(";");
  Serial.println(current, 2); 
}

void test_motors()
{
  if(armed == false)
  return;
  
  if(test == false)
  return;

  motor.writeMicroseconds(pwm_value);

  // Increase pwm for next reading
  if(pwm_value <= pwm_high)
  {
    pwm_value = pwm_value + pwm_step;
  }
  else
  {
    motor.writeMicroseconds(pwm_low);
    pwm_value = pwm_arm;
    test = false;
  }
}

void setup() 
{
  Serial.begin(uart_baud);

  thrust.set_scale(load_cell_calib);
  torque_A.set_scale(load_cell_calib);
  torque_B.set_scale(load_cell_calib);
  
  thrust.tare();
  torque_A.tare();
  torque_B.tare();

  motor.attach(pwm_pin);
  motor.writeMicroseconds(0);

  voltage = analogRead(voltage_pin) * (5.0 / 1024) * voltage_divider_ratio;
  current = 0.0;
}

void loop() 
{
  check_serial();

  read_sensors();

  test_motors();
  
  delay(loop_delay);
}
