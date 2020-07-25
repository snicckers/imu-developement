#include <Wire.h>
#include <stdlib.h>
#include <Servo.h>
#include <IRremote.h>

#define ACTIVATED HIGH
unsigned long elapsed_time;
unsigned long last_time_print;
/*--- Propeller Servos -------------------------------------------------------*/
Servo right_prop;
Servo left_prop;
double throttle = 1100;
int button_state = 0;
int previous_time_pressed;
bool start_motors = false;

//--- Simple Moving Average Globals ------------------------------------------*/
const int samples = 10;
int a_x_readings[samples];
int a_y_readings[samples];
int a_z_readings[samples];
long int a_read_index = 0;
long int a_read_total[3] = {0, 0, 0};
long int a_read_ave[3] = {0, 0, 0};

/*--- Time Control -----------------------------------------------------------*/
int refresh_rate = 250;
float dt = 1 / refresh_rate;
const float loop_micros = (dt) * 1000000;

/*--- IMU Globals ------------------------------------------------------------*/
float rad_to_degrees = 57.29577951;
float degrees_to_rad = 0.017453293;
float a_magnitude, a_roll, a_pitch;
float g_roll, g_pitch;
float g_modifier = (1/65.5)*(dt);
float g_modifier_rads = g_modifier * 0.017453293;
float i_roll = 0, i_pitch = 0;
long g_cal[3];

/*--- PID Globals ------------------------------------------------------------*/
float pid, pwm_right, pwm_left, error, previous_error;
float pid_p = 0, pid_i = 0, pid_d = 0;
float k_p = 1.4;
float k_i = 1.82;
float k_d = 1.04;
float desired_angle = 0.0;

/*--- Remote Control ---------------------------------------------------------*/
IRrecv irrecv(12);    // IR reciver digital input to pin 12.
decode_results results;

/*--- setup mpu --------------------------------------------------------------*/
void setup_mpu(){
  // Activate the MPU-6050
  // 0x68 = Registry address of mpu6050
  // 0x6B = Send starting register
  // 0x00 = Tell the MPU not to be asleep
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  // Configure the accelerometer (+/-8g)
  // 0x68 = Registry address of mpu6050
  // 0x1C = Registry address of accelerometer
  // 0x10 = Full scale range of accelerometer (data sheet)
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  // Configure the gyro (500dps full scale
  // 0x68 = Registry address of mpu6050
  // 0x1B = Registry address of gyroscope
  //        0x08 = 500 degree / sec Range of the gyro in degree/sec (data sheet)
  //        0x10 = 1000 degree / sec range
  // 0x12 = 2000 degree / sec range
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x10);
  Wire.endTransmission();
}

/*--- read mpu  --------------------------------------------------------------*/
void read_mpu(int ** sensor_output_array){

  int array_size = 10;
  *sensor_output_array = (int*) malloc(sizeof(int) * array_size);
  /* Access the accellerometer register and requst
  14 bits. Assign each high and low bit to a variable. */
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 14);
  while(Wire.available() < 14){}; // Wait for all of the bits to be recieved:
  // Assign values to each element of the array:
  (*sensor_output_array)[0] = Wire.read()<<8|Wire.read(); // a_x
  (*sensor_output_array)[1] = Wire.read()<<8|Wire.read(); // a_y
  (*sensor_output_array)[2] = Wire.read()<<8|Wire.read(); // a_z
  (*sensor_output_array)[3] = Wire.read()<<8|Wire.read(); // temp
  (*sensor_output_array)[4] = Wire.read()<<8|Wire.read(); // g_x
  (*sensor_output_array)[5] = Wire.read()<<8|Wire.read(); // g_y
  (*sensor_output_array)[6] = Wire.read()<<8|Wire.read(); // g_z
}

/*--- DATA PROCESSING --------------------------------------------------------*/
void accel_data_processing(int * sensor_data[]){  //Simple moving average filter
  a_read_total[0] -= a_x_readings[a_read_index];
  a_read_total[1] -= a_y_readings[a_read_index];
  a_read_total[2] -= a_z_readings[a_read_index];
  a_x_readings[a_read_index] = (*sensor_data)[0];
  a_y_readings[a_read_index] = (*sensor_data)[1];
  a_z_readings[a_read_index] = (*sensor_data)[2];
  a_read_total[0] += a_x_readings[a_read_index];
  a_read_total[1] += a_y_readings[a_read_index];
  a_read_total[2] += a_z_readings[a_read_index];
  a_read_index += 1;
  if (a_read_index >= samples){
    a_read_index = 0;
  }
  a_read_ave[0] = a_read_total[0] / samples;
  a_read_ave[1] = a_read_total[1] / samples;
  a_read_ave[2] = a_read_total[2] / samples;
}

void gyro_data_processing(int * sensor_data[]){
  (*sensor_data)[4] -= g_cal[0];
  (*sensor_data)[5] -= g_cal[1];
  (*sensor_data)[6] -= g_cal[2];
}

/*--- Calculate Roll Pitch & Yaw ---------------------------------------------*/
void calculate_attitude(int sensor_data[]){
  /*--- Attitude from accelerometer -------------*/
  a_magnitude = sqrt(pow(a_read_ave[0], 2) + pow(a_read_ave[1], 2) + pow(a_read_ave[2], 2));
  a_roll = asin((float)a_read_ave[1] / a_magnitude) * rad_to_degrees; // X
  a_pitch = asin((float)a_read_ave[0] / a_magnitude) * rad_to_degrees * (-1); // Y
  // Attitude correction offset
  a_roll -= 1.2;
  a_pitch += 3.2;

  /*--- Attitude from Gyroscope ----------------------------------------------*/
  //0.000061069
  float t = (micros() - elapsed_time);
  elapsed_time = micros();
  double lsb_coefficient = (1 / 32.8) * ((t) / 1000000);
  g_roll += sensor_data[4] * lsb_coefficient * 1.09;
  g_pitch += sensor_data[5] * lsb_coefficient * 1.09;
  //1.09 = coefficient that makes gyro attitue match accel attitude.
  /* Approximation for transfer of roll to pitch & vice versa using yaw. Thanks
  Joop: */
  g_roll += g_pitch * sin(sensor_data[6] * lsb_coefficient * 0.0174533 * 0.5);
  g_pitch -= g_roll * sin(sensor_data[6] * lsb_coefficient * 0.0174533 * 0.5);

  /*--- Complimentary FIlter --------------------*/
  // Apply complimentary filter if force magnitude is within reasonable range.
  int f_magnitude_approx = abs(a_read_ave[0]) + abs(a_read_ave[1]) + abs(a_read_ave[2]);
  if (f_magnitude_approx  > 3072 && f_magnitude_approx < 32768){ // 4096 = 1g for +- 8G range
    float hpf = 0.9996; // High Pass Filter
    float lpf = 1.0 - hpf; // Low Pass Fbilter
    g_roll = hpf * g_roll + lpf * a_roll;
    g_pitch = hpf * g_pitch + lpf * a_pitch;
  }
}

/*--- Calibrate IMU ----------------------------------------------------------*/
void calibrate_imu(){
  /* THE IMU MUST NOT BE MOVED DURING SETUP */
  /*--- Simple Moving ave Setup ---*/
  for (int i = 0; i < samples; i++){
    a_x_readings[i] = 0;
    a_y_readings[i] = 0;
    a_z_readings[i] = 0;
  }

  /*--- Calibrate gyroscope data and initial attitude: ---*/
  int cal_count = 750;
  Serial.print("\nCalibrating \n");
  for (int i = 0; i < cal_count; i ++){
    // Print the loading bar blips n times
    if(i % 125 == 0) { Serial.print("- "); }
    // Collect data from MPU
    int * data_xyzt; read_mpu(&data_xyzt);
    g_cal[0] += data_xyzt[4];
    g_cal[1] += data_xyzt[5];
    g_cal[2] += data_xyzt[6];
    accel_data_processing(&data_xyzt);
    calculate_attitude(data_xyzt);
    i_roll += a_roll;
    i_pitch += a_pitch;
    free(data_xyzt); // Clear dynamic memory allocation
    delay(3);
  }
  // Find the average value of the data that was recorded above:
  g_cal[0] /= cal_count;
  g_cal[1] /= cal_count;
  g_cal[2] /= cal_count;
  i_roll /= cal_count;
  i_pitch /= cal_count;

  // Set initial angle based off accelerometer
  g_roll = i_roll;
  g_pitch = i_pitch;
}

/*--- Flight Controller ------------------------------------------------------*/
void flight_controller(){
  error = desired_angle - g_roll;

  // PROPORTIONAL COMPONENET
  pid_p = k_p * error;

  // INTEGRAL COMPONENT
  int k_i_thresh = 8;
  if (error < k_i_thresh && error > -k_i_thresh) {
    pid_i = pid_i * (k_i * error);
  }
  if (error > k_i_thresh && error < -k_i_thresh){
    pid_i = 0;
  }

  /* DERIVATIVE COMPONENT*/
  pid_d = k_d * ((error - previous_error) / 0.004);

  /* Sum the the components to find the total pid value. */
  pid = pid_p + pid_i + pid_d;

  /* Clamp the maximum & minimum pid values*/
  if (pid < -1000){
    pid = -1000;
  }
  if (pid > 1000){
    pid = 1000;
  }

  /* Calculate PWM width. */
  pwm_right = throttle + pid;
  pwm_left = throttle - pid;

  /* clamp the PWM values. */
  //----------Right---------//
  if (pwm_right < 1000){
    pwm_right = 1000;
  }
  if (pwm_right > 2000){
    pwm_right = 2000;
  }
    //----------Left---------//
  if (pwm_left < 1000){
    pwm_left = 1000;
  }
  if (pwm_left > 2000){
    pwm_left = 2000;
  }

  if (start_motors == true){
    right_prop.writeMicroseconds(pwm_right);
    left_prop.writeMicroseconds(pwm_left);
  } else{
    right_prop.writeMicroseconds(1000);
    left_prop.writeMicroseconds(1000);
  }

  previous_error = error;
}

/*--- Debugging --------------------------------------------------------------*/
void debugging(){
//   Serial.println(0);
//   Serial.print("\n");
//   Serial.println(5000);
//   Serial.print(" ");
//   Serial.print(loop_time);
//   Serial.print(" ");
//   Serial.print(loop_micros);
//   Serial.print(" ");
//   Serial.print("\n");

  /*--- Print ---*/
  // Serial.print("\n");
  // Serial.print(90);
  // Serial.print(" ");
  // Serial.print(-90);
  // Serial.print(" ");

  int mode = 2;

  if (elapsed_time - last_time_print > 35000){
    if(mode == 1){
        Serial.print("Roll: ");
      Serial.print(g_roll);

      Serial.print(" - Pitch: ");
      Serial.print(g_pitch);

      Serial.print(" - pwm left: ");
      Serial.print(pwm_left);

      Serial.print(" - pwm right: ");
      Serial.print(pwm_right);

      Serial.print(" - PID: ");
      Serial.print(pid);

      Serial.print(" - Run Motors?: ");
      Serial.print(start_motors);

      Serial.print(" - k_p: ");
      Serial.print(k_p);
      Serial.print(" - k_i: ");
      Serial.print(k_i);
      Serial.print(" - k_d: ");
      Serial.print(k_d);

      Serial.print("\n");
    }
    if(mode == 2){
    //  Serial.print("");
    //  Serial.print();
      Serial.print(" aPitch: ");
      Serial.print(a_pitch);
      Serial.print(" - gPitch: ");
      Serial.print(g_pitch);

      Serial.print(" - aRoll: ");
      Serial.print(a_roll);
      Serial.print(" - gRoll: ");
      Serial.print(g_roll);

      Serial.print(" ");
      Serial.print(90);
      Serial.print(" ");
      Serial.print(-90);

      Serial.print("\n");
    }
    if(mode == 3){
      // Serial.print(" gRoll: ");
      // Serial.print(g_roll);
      // Serial.print(" - gPitch: ");
      Serial.print(g_pitch);
      // Serial.print(" ");
      // Serial.print(90);
      // Serial.print(" ");
      // Serial.print(-90);
      // Serial.print(" - k_p: ");
      // Serial.print(k_p);
      // Serial.print(" - k_i: ");
      // Serial.print(k_i);
      // Serial.print(" - k_d: ");
      // Serial.print(k_d);


      Serial.print("\n");
    }


    last_time_print = micros();
  }
}

void motors_on_off(){
  button_state = digitalRead(13);
  long int elapsed_time = millis();
  if (button_state == ACTIVATED && start_motors == false && ((elapsed_time - previous_time_pressed) > 700)){
    start_motors = true;
    previous_time_pressed = millis();
  }
  else if(button_state == ACTIVATED && start_motors == true && ((elapsed_time - previous_time_pressed) > 700)){
    start_motors = false;
    previous_time_pressed = millis();
  }
}

void IR_remoteControl(){
  /*--- Store IR reciever remote value ---*/
  if(irrecv.decode(&results)){
    if(results.value != 4294967295){
      //Serial.println(results.value, HEX);

      /*--- Change PID gain values ---*/
      switch(results.value){

        case 1320906895:    // Power Button:
          if (start_motors){
            start_motors = false;
          } else{
            start_motors = true;
          }
          break;

        case 1320929335:    // Button 1
          k_p += 0.02;
          break;

        case 1320880375:    // Button 2
          k_d += 0.02;
          break;

        case 1320913015:    // Button 3
          k_i += 0.02;
          break;

        case 1320939535:    // Button 4
          k_p -= 0.02;
          break;

        case 1320890575:    // Button 5
          k_d -= 0.02;
          break;

        case 1320923215:    // Button 6
          k_i -= 0.02;
          break;

        case 1320887005:    // Up Button
          throttle += 50;
          break;

        case 1320925255:
          throttle -= 50;   // Down Button
          break;

        default:
          break;
      }
    }
    irrecv.resume();
  }
}

void potentiometer_control(){
  float value = analogRead(A0);
  value = map(value, 1, 1024, 30, -30);
  desired_angle = value;
}

/*--- Setup ------------------------------------------------------------------*/
void setup() {
  Serial.begin(2000000);
  Wire.begin();
  irrecv.enableIRIn();
  // Motors
  right_prop.attach(5);
  left_prop.attach(3);
  right_prop.writeMicroseconds(1000);
  left_prop.writeMicroseconds(1000);
  // Calibrate imu
  setup_mpu();
  calibrate_imu();
}

/*--- Main -------------------------------------------------------------------*/
void loop(){
  //IMU
  int * data_xyzt;
  read_mpu(&data_xyzt);
  accel_data_processing(&data_xyzt);
  gyro_data_processing(&data_xyzt);
  calculate_attitude(data_xyzt);
  free(data_xyzt);  
  // FLIGHT CONTROLLER
  flight_controller();
  // DEBUGGING
  debugging();
  //CALIBRATION CONTROLS
  IR_remoteControl();
  potentiometer_control();
  //TIME CONTROL
  while (micros() - elapsed_time < 4000);
}
