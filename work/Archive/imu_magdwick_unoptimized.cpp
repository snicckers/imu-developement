#include <Wire.h>
#include <stdlib.h>
#include <Servo.h>
#include <IRremote.h>

#define ACTIVATED HIGH
unsigned long elapsed_time;
float sample_time;
unsigned long last_time_print;
/*--- Propeller Servos -------------------------------------------------------*/
Servo right_prop;
Servo left_prop;
double throttle = 1100;
int button_state = 0;
int previous_time_pressed;
bool start_motors = false;

//--- Simple Moving Average Globals ------------------------------------------*/
const int samples = 5;
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
float rad_to_degrees = 57.29577951f;
float degrees_to_rad = 0.017453293f;
double lsb_coefficient = (1.0f / 32.8f);
float roll, pitch, yaw;
float initial_roll, initial_pitch, initial_yaw;
long g_cal[3];

float qi_0, qi_1, qi_2, qi_3;
float q_0 = 1.0f;
float q_1 = 0.0f;
float q_2 = 0.0f;
float q_3 = 0.0f;
float correction_gain = 0.2f;

/*--- PID Globals ------------------------------------------------------------*/
float pid, pwm_right, pwm_left, error, previous_error, previous_roll;
float pid_p = 0, pid_i = 0, pid_d = 0;
float k_p = 3.78f; //3.5
float k_i = 0.05f; //
float k_d = 1.03f;  //0.85
// float k_p = 1.4;
// float k_i = 1.82;
// float k_d = 1.04;
float desired_angle = 0.0;

/*--- REMOTE CONTROL ---------------------------------------------------------*/
IRrecv irrecv(12);    // IR reciver digital input to pin 12.
decode_results results;
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
unsigned long timer_1, timer_2, timer_3, timer_4;

/*--- DEBUGGING --------------------------------------------------------------*/
void debugging(){
  int mode = 1;

  if (elapsed_time - last_time_print > 20000){
    if(mode == 1){
        Serial.print("Roll: ");
      Serial.print(roll);

      Serial.print(" - Pitch: ");
      Serial.print(pitch);

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
      // Serial.print(" aPitch: ");
      // Serial.print(a_pitch);
      // Serial.print("gPitch: ");
      Serial.print(pitch);
      Serial.print(" ");
      // Serial.print(" - aRoll: ");
      // Serial.print(a_roll);
      // Serial.print(" - gRoll: ");
      Serial.print(roll);
      // Serial.print(" ");
      // Serial.print(90);
      // Serial.print(" ");
      // Serial.print(-90);
      Serial.print("\n");
    }
    if(mode == 3){
      // Serial.print(" gRoll: ");
      // Serial.print(roll);
      // Serial.print(" - gPitch: ");
      Serial.print(pitch);
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

void debug_loopTime(){
  if (elapsed_time - last_time_print > 100000){
    Serial.print(micros() - elapsed_time);
    Serial.print("\n");
    last_time_print = micros();
  }
}

/*--- SETUP MPU --------------------------------------------------------------*/
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

/*--- READ MPU  --------------------------------------------------------------*/
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

/*--- CALCULATE ATTITUDE -----------------------------------------------------*/
float invSqrt( float number ){
    union {
        float f;
        uint32_t i;
    } conv;

    float x2;
    const float threehalfs = 1.5F;

    x2 = number * 0.5F;
    conv.f  = number;
    conv.i  = 0x5f3759df - ( conv.i >> 1 );
    conv.f  = conv.f * ( threehalfs - ( x2 * conv.f * conv.f ) );
    return conv.f;
}

void calculate_initial_attitude(int sensor_data[]){
  float a_magnitude = sqrt(a_read_ave[0] * a_read_ave[0] + a_read_ave[1] * a_read_ave[1] + a_read_ave[2] * a_read_ave[2]);
  initial_roll += asin((float)a_read_ave[1] / a_magnitude) * rad_to_degrees; // X
  initial_pitch += asin((float)a_read_ave[0] / a_magnitude) * rad_to_degrees * (-1.0f);
}

void calculate_attitude(int sensor_data[]){
  /*--- Madgwick Filter ------------------------------------------------------*/
  float normalize;

  float a_x = sensor_data[0];
  float a_y = sensor_data[1];
  float a_z = sensor_data[2];
  normalize = invSqrt(a_x*a_x + a_y*a_y + a_z*a_z);
  a_x *= normalize;
  a_y *= normalize;
  a_z *= normalize;

  float a_magnitude = sqrt(a_read_ave[0] * a_read_ave[0] + a_read_ave[1] * a_read_ave[1] + a_read_ave[2] * a_read_ave[2]);
  initial_roll += asin((float)a_read_ave[1] / a_magnitude) * rad_to_degrees; // X
  initial_pitch += asin((float)a_read_ave[0] / a_magnitude) * rad_to_degrees * (-1.0f);

  // 1.09 = fudge factor. g_x in radians / sec
  float g_x = sensor_data[4] * (lsb_coefficient) * (1.00) * degrees_to_rad;
  float g_y = sensor_data[5] * (lsb_coefficient) * (1.00) * degrees_to_rad;
  float g_z = sensor_data[6] * (lsb_coefficient) * (1.00) * degrees_to_rad;

  // q_dot = 0.5 angular velocity rotation maxtrix * q.
  // Reference: A New Quaternion-Based Kalman Filter for Real-Time Attitude Estimation Using the Two-Step Geometrically-Intuitive Correction Algorithm. Equation 32 in section 2.3.1

  float qDot_0 = 0.5f*(-q_1*g_x - q_2*g_y - q_3*g_z);
  float qDot_1 = 0.5f*(q_0*g_x + q_2*g_z - q_3*g_y);
  float qDot_2 = 0.5f*(q_0*g_y + q_2*g_x - q_1*g_z);
  float qDot_3 = 0.5f*(q_0*g_z + q_1*g_y - q_2*g_x);

  // If accelerometer is working carry out gradient descent algorithm.
  if(!((a_x == 0.0f) && (a_y == 0.0f) && (a_z == 0.0f))) {

    float delF_0 =
    4.0f*q_2*q_2*q_0 +
    4.0f*q_0*q_1*q_1 +
    2.0f*q_2*a_x -
    2.0f*q_1*a_y;

    float delF_1 =
    8.0f*q_1*q_1*q_1 +
    4.0f*q_3*q_3*q_1 +
    4.0f*q_0*q_0*q_1 -
    4.0f*q_1 +
    8.0f*q_2*q_2*q_1 -
    2.0f*q_3*a_x -
    2.0f*q_0*a_y +
    4.0f*q_1*a_z;

    float delF_2 =
    8.0f*q_2*q_2*q_2 -
    4.0f*q_2 +
    4.0f*q_2*q_3*q_3 +
    4.0f*q_2*q_0*q_0 +
    8.0f*q_2*q_1*q_1 +
    2.0f*q_0*a_x -
    2.0f*q_3*a_y +
    4.0f*q_2*a_z;

    float delF_3 =
    4.0f*q_2*q_2*q_3 +
    4.0f*q_3*q_1*q_1 -
    2.0f*q_1*a_x -
    2.0f*q_2*a_y;

    normalize = invSqrt(delF_0*delF_0 + delF_1*delF_1 + delF_2*delF_2 + delF_3*delF_3);
    delF_0 *= normalize;
    delF_1 *= normalize;
    delF_2 *= normalize;
    delF_3 *= normalize;

    // Change correction_gain for more or less influence on gyro rates.
    qDot_0 -= correction_gain * delF_0;
    qDot_1 -= correction_gain * delF_1;
    qDot_2 -= correction_gain * delF_2;
    qDot_3 -= correction_gain * delF_3;
  }

  q_0 += qDot_0 * sample_time;
  q_1 += qDot_1 * sample_time;
  q_2 += qDot_2 * sample_time;
  q_3 += qDot_3 * sample_time;

  roll = atan2f(2*(q_0*q_1 + q_2*q_3), 1.0f - 2.0f*(q_1*q_1 + q_2*q_2)) * rad_to_degrees;
  pitch = asinf(2.0f * (q_0*q_2 - q_1*q_3)) * rad_to_degrees;
  //yaw = atan2f(2*(q_0*q_3 + q_1*q_2), 1.0f - 2.0f*(q_2*q_2 + q_3*q_3)) * rad_to_degrees;
}

/*--- CALIBRATE IMU ----------------------------------------------------------*/
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
    sample_time = (micros() - elapsed_time) / 1000000.0f;
    elapsed_time = micros();
    // Print the loading bar blips n times
    if(i % 50 == 0) { Serial.print("-"); }
    // Collect data from MPU
    int * data_xyzt;
    read_mpu(&data_xyzt);
    g_cal[0] += data_xyzt[4];
    g_cal[1] += data_xyzt[5];
    g_cal[2] += data_xyzt[6];
    accel_data_processing(&data_xyzt);
    calculate_initial_attitude(data_xyzt);
    free(data_xyzt); // Clear dynamic memory allocation
    delay(3);
  }
  // Find the average value of the data that was recorded above:
  g_cal[0] /= cal_count;
  g_cal[1] /= cal_count;
  g_cal[2] /= cal_count;
  initial_pitch /= cal_count;
  initial_roll /= cal_count;
  initial_yaw = 0;

  float c1, c2, c3, s1, s2, s3;

  c1 = cos(initial_yaw / 2);
  c2 = cos(initial_pitch / 2);
  c3 = cos(initial_roll / 2);
  s1 = sin(initial_yaw / 2);
  s2 = sin(initial_pitch / 2);
  s3 = sin(initial_roll / 2);
  q_0 = c1*c2*c3 - s1*s2*s3;
  q_1 = s1*s2*c3 + c1*c2*s3;
  q_2 = s1*c2*c3 + c1*s2*s3;
  q_3 = c1*s2*c3 - s1*c2*s3;
}

/*--- FLIGHT CONTROLLER ------------------------------------------------------*/
void flight_controller(){
  error = desired_angle - roll;

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
  // Derivitive of the process variable (roll), NOT THE ERROR
  // Taking derivative of the error results in "Derivative Kick".
  // https://www.youtube.com/watch?v=KErYuh4VDtI
  pid_d = (-1.0f) * k_d * ((roll - previous_roll) / sample_time);
  // pid_d = k_d * ((error - previous_error) / sample_time);
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
  previous_roll = roll;
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
          k_p += 0.05;
          break;

        case 1320880375:    // Button 2
          k_d += 0.05;
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

void change_setpoint(){

  if (receiver_input_channel_1 != 0){
    desired_angle = map(receiver_input_channel_1, 1000, 2000, 30, -30);
  }
}

void setup_interrupts(){
  // put your setup code here, to run once:
  PCICR |= (1 << PCIE0);  // Set OCIE0 to enable PCMSK0 to scan
  PCMSK0 |= (1 << PCINT0);  // set digital input 8 to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT1);  // etc
  PCMSK0 |= (1 << PCINT2);
  PCMSK0 |= (1 << PCINT3);
}

/*--- SETUP ------------------------------------------------------------------*/
void setup() {
  pinMode(7, INPUT);
  setup_interrupts();
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


/*--- MAIN -------------------------------------------------------------------*/
void loop(){
  sample_time = (micros() - elapsed_time) / 1000000.0f;
  elapsed_time = micros();
  //IMU
  int * data_xyzt;
  change_setpoint();
  read_mpu(&data_xyzt);
  accel_data_processing(&data_xyzt);
  gyro_data_processing(&data_xyzt);
  calculate_attitude(data_xyzt);
  free(data_xyzt);  // Clear allocated memory for data array.
  // FLIGHT CONTROLLER
  roll = roll - 5;
  flight_controller();
  // DEBUGGING
  debugging();
  //CALIBRATION CONTROLS
  IR_remoteControl();
  //debug_loopTime();
  // REFRESH RATE
  while (micros() - elapsed_time < 5500);
  // if (micros() - elapsed_time > 5500){  //Freeze if the loop takes too long
  //   while(true);
  // }
}

ISR(PCINT0_vect){
  /*----- CHANNEL 1 -----*/
  if(last_channel_1 == 0 && PINB & B00000001){
    last_channel_1 = 1;
    timer_1 = micros();
  }
  else if(last_channel_1 == 1 && !(PINB & B00000001)){
    last_channel_1 = 0;
    receiver_input_channel_1 = micros() - timer_1;
  }
}
