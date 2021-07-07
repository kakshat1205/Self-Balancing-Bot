
#include <Wire.h>
int gyro_address = 0x68;
int acc_calibration_value = -7979;
int r = 0, l = 0, d = 0;
float pid_p_gain = 15;
float pid_i_gain = 0.5;
float pid_d_gain = 15;
float turning_speed = 30;
float max_target_speed = 150;

byte start, received_byte, low_bat;

int left_motor, throttle_left_motor, throttle_counter_left_motor, throttle_left_motor_memory;
int right_motor, throttle_right_motor, throttle_counter_right_motor, throttle_right_motor_memory;

int receive_counter;
int gyro_pitch_data_raw, gyro_yaw_data_raw, accelerometer_data_raw;

long gyro_yaw_calibration_value, gyro_pitch_calibration_value;

unsigned long loop_timer;

float angle_gyro, angle_acc, angle, self_balance_pid_setpoint;
float pid_error_temp, pid_i_mem, pid_setpoint, gyro_input, pid_output, pid_last_d_error;
float pid_output_left, pid_output_right;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  TWBR = 12;
  TCCR2A = 0;
  TCCR2B = 0;
  TIMSK2 |= (1 << OCIE2A);
  TCCR2B |= (1 << CS21);
  OCR2A = 39;
  TCCR2A |= (1 << WGM21);

  Wire.beginTransmission(gyro_address);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(gyro_address);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(gyro_address);
  Wire.write(0x1C);
  Wire.write(0x08);
  Wire.endTransmission();

  Wire.beginTransmission(gyro_address);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission();

  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(13, OUTPUT);

  for (receive_counter = 0; receive_counter < 500; receive_counter++) {
    if (receive_counter % 15 == 0)digitalWrite(13, !digitalRead(13));
    Wire.beginTransmission(gyro_address);
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom(gyro_address, 4);
    gyro_yaw_calibration_value += Wire.read() << 8 | Wire.read();
    gyro_pitch_calibration_value += Wire.read() << 8 | Wire.read();
    delayMicroseconds(3700);
  }
  gyro_pitch_calibration_value /= 500;
  gyro_yaw_calibration_value /= 500;

  loop_timer = micros() + 4000;
}

void loop() {
  if (Serial.available()) {
    received_byte = Serial.read();
    receive_counter = 0;
  }
  if (receive_counter <= 25)receive_counter ++;
  else received_byte = 0x00;


  Wire.beginTransmission(gyro_address);
  Wire.write(0x3F);
  Wire.endTransmission();
  Wire.requestFrom(gyro_address, 2);
  accelerometer_data_raw = Wire.read() << 8 | Wire.read();
  accelerometer_data_raw += acc_calibration_value;
  if (accelerometer_data_raw > 8200)accelerometer_data_raw = 8200;
  if (accelerometer_data_raw < -8200)accelerometer_data_raw = -8200;

  angle_acc = asin((float)accelerometer_data_raw / 8200.0) * 57.296;

  if (start == 0 && angle_acc > -0.5 && angle_acc < 0.5) {
    angle_gyro = angle_acc;
    start = 1;
  }

  Wire.beginTransmission(gyro_address);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(gyro_address, 4);
  gyro_yaw_data_raw = Wire.read() << 8 | Wire.read();
  gyro_pitch_data_raw = Wire.read() << 8 | Wire.read();

  gyro_pitch_data_raw -= gyro_pitch_calibration_value;
  angle_gyro += gyro_pitch_data_raw * 0.000031;


  gyro_yaw_data_raw -= gyro_yaw_calibration_value;


  angle_gyro = angle_gyro * 0.9996 + angle_acc * 0.0004;
  pid_error_temp = angle_gyro - self_balance_pid_setpoint - pid_setpoint;
  if (pid_output > 10 || pid_output < -10)pid_error_temp += pid_output * 0.015 ;

  pid_i_mem += pid_i_gain * pid_error_temp;
  if (pid_i_mem > 400)pid_i_mem = 400;
  else if (pid_i_mem < -400)pid_i_mem = -400;

  pid_output = pid_p_gain * pid_error_temp + pid_i_mem + pid_d_gain * (pid_error_temp - pid_last_d_error);
  if (pid_output > 400)pid_output = 400;
  else if (pid_output < -400)pid_output = -400;

  pid_last_d_error = pid_error_temp;

  if (pid_output < 5 && pid_output > -5)pid_output = 0;
  if (angle_gyro > 30 || angle_gyro < -30 || start == 0 || low_bat == 1) {
    pid_output = 0;
    pid_i_mem = 0;
    start = 0;
    self_balance_pid_setpoint = 0;
  }

  pid_output_left = pid_output;                                             //Copy the controller output to the pid_output_left variable for the left motor
  pid_output_right = pid_output;                                            //Copy the controller output to the pid_output_right variable for the right motor

  if (r == 1) {                                        //If the first bit of the receive byte is set change the left and right variable to turn the robot to the left
    pid_output_left += turning_speed;                                       //Increase the left motor speed
    pid_output_right -= turning_speed;
//    digitalWrite(led1 , r);//Decrease the right motor speed
  }
  else if (l == 1) {                                        //If the second bit of the receive byte is set change the left and right variable to turn the robot to the right
    pid_output_left -= turning_speed;                                       //Decrease the left motor speed
    pid_output_right += turning_speed;
//    digitalWrite(led1 , l);//Increase the right motor speed
    //Decrease the self_balance_pid_setpoint if the robot is still moving backwards
  }

  else if (d == 0) {
    if (pid_setpoint > -2.5)pid_setpoint -= 0.05;                           //Slowly change the setpoint angle so the robot starts leaning forewards
    if (pid_output > max_target_speed * -1)pid_setpoint -= 0.005;
//    digitalWrite(led1 , d);//Decrease the self_balance_pid_setpoint if the robot is still moving backwards
  }


  if (d != 0) {                                     
    if (pid_setpoint > 0.5)pid_setpoint -= 0.05;                            
    else if (pid_setpoint < -0.5)pid_setpoint += 0.05;                      
    else pid_setpoint = 0;
  }
  //The self balancing point is adjusted when there is not forward or backwards movement from the transmitter. This way the robot will always find it's balancing point
  if (pid_setpoint == 0) {                                                  //If the setpoint is zero degrees
    if (pid_output < 0)self_balance_pid_setpoint += 0.0015;                 //Increase the self_balance_pid_setpoint if the robot is still moving forewards
    if (pid_output > 0)self_balance_pid_setpoint -= 0.0015;                 //Decrease the self_balance_pid_setpoint if the robot is still moving backwards
  }

  if (pid_output_left > 0)pid_output_left = 405 - (1 / (pid_output_left + 9)) * 5500;
  else if (pid_output_left < 0)pid_output_left = -405 - (1 / (pid_output_left - 9)) * 5500;

  if (pid_output_right > 0)pid_output_right = 405 - (1 / (pid_output_right + 9)) * 5500;
  else if (pid_output_right < 0)pid_output_right = -405 - (1 / (pid_output_right - 9)) * 5500;
  if (pid_output_left > 0)left_motor = 400 - pid_output_left;
  else if (pid_output_left < 0)left_motor = -400 - pid_output_left;
  else left_motor = 0;

  if (pid_output_right > 0)right_motor = 400 - pid_output_right;
  else if (pid_output_right < 0)right_motor = -400 - pid_output_right;
  else right_motor = 0;

  throttle_left_motor = left_motor;
  throttle_right_motor =  right_motor;

  while (loop_timer > micros());
  loop_timer += 4000;
}

ISR(TIMER2_COMPA_vect) {

  throttle_counter_left_motor ++;                                           //Increase the throttle_counter_left_motor variable by 1 every time this routine is executed
  if (throttle_counter_left_motor > throttle_left_motor_memory) {           //If the number of loops is larger then the throttle_left_motor_memory variable
    throttle_counter_left_motor = 0;                                        //Reset the throttle_counter_left_motor variable
    throttle_left_motor_memory = throttle_left_motor;                       //Load the next throttle_left_motor variable
    if (throttle_left_motor_memory < 0) {                                   //If the throttle_left_motor_memory is negative
      PORTH &= 0b10111111;                                                  //Set output 3 low to reverse the direction of the stepper controller
      throttle_left_motor_memory *= -1;                                     //Invert the throttle_left_motor_memory variable
    }
    else PORTH |= 0b01000000;                                               //Set output 3 high for a forward direction of the stepper motor
  }
  else if (throttle_counter_left_motor == 1)PORTH |= 0b00100000;            //Set output 2 high to create a pulse for the stepper controller
  else if (throttle_counter_left_motor == 2)PORTH &= 0b11011111;            //Set output 2 low because the pulse only has to last for 20us

  //right motor pulse calculations
  throttle_counter_right_motor ++;                                          //Increase the throttle_counter_right_motor variable by 1 every time the routine is executed
  if (throttle_counter_right_motor > throttle_right_motor_memory) {         //If the number of loops is larger then the throttle_right_motor_memory variable
    throttle_counter_right_motor = 0;                                       //Reset the throttle_counter_right_motor variable
    throttle_right_motor_memory = throttle_right_motor;                     //Load the next throttle_right_motor variable
    if (throttle_right_motor_memory < 0) {                                  //If the throttle_right_motor_memory is negative
      PORTB |= 0b00100000;                                                  //Set output 5 low to reverse the direction of the stepper controller
      throttle_right_motor_memory *= -1;                                    //Invert the throttle_right_motor_memory variable
    }
    else PORTB &= 0b11011111;                                               //Set output 5 high for a forward direction of the stepper motor
  }
  else if (throttle_counter_right_motor == 1)PORTB |= 0b00010000;           //Set output 4 high to create a pulse for the stepper controller
  else if (throttle_counter_right_motor == 2)PORTB &= 0b11101111;           //Set output 4 low because the pulse only has to last for 20us
}
