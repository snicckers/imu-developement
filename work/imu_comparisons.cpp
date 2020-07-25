#include <Wire.h>
#include <stdlib.h>
#include <Servo.h>
#include <IRremote.h>

unsigned long elapsed_time;
float sample_time;
unsigned long last_time_print;

//--- Simple Moving Average Globals ------------------------------------------*/
const int samples = 15;
int a_x_readings[samples];
int a_y_readings[samples];
int a_z_readings[samples];
long int a_read_index = 0;
long int a_read_total[3] = {0, 0, 0};
long int a_read_ave[3] = {0, 0, 0};

/*--- IMU Globals ------------------------------------------------------------*/
float rad_to_degrees = 57.29577951f;
float degrees_to_rad = 0.017453293f;
double lsb_coefficient = (1.0f / 32.8f); // see datasheet
float roll, pitch, yaw;
long g_drift[3];
float q_0 = 1.0f;
float q_1 = 0.0f;
float q_2 = 0.0f;
float q_3 = 0.0f;
float q0 = 1.0f;
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;
float correction_gain = 0.2f;
int imu_mode = 0;

float a_magnitude, a_roll, a_pitch;
float g_roll, g_pitch;
float g_modifier = 0.000076336;
float g_modifier_rads = g_modifier * 0.017453293;
float i_roll = 0, i_pitch = 0;
long g_cal[3];

// Time taken for each imu calculation pass
long time_1, time_2, time_3, time_4, time_5;

/*--- DEBUGGING --------------------------------------------------------------*/
// This method prints the time taken from the beginning of the scan to the time this method is envocked. In order not to kill performance, this is only printed every idk 100000 microseconds.
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

// Remove the average gyroscope drift / offset (recorded in the calibration method) from the gyroscope data that is recorded during each scan.
void gyro_data_processing(int * sensor_data[]){
  (*sensor_data)[4] -= g_drift[0];
  (*sensor_data)[5] -= g_drift[1];
  (*sensor_data)[6] -= g_drift[2];
}

/*--- CALCULATE ATTITUDE -----------------------------------------------------*/
// A cheap way to find the inverse squareroot of a number.
float invSqrt( float x ){
    float xhalf = 0.5f*x;
    union {
        float x;
        int i;
    } u;
    u.x = x;
    u.i = 0x5f375a86 - (u.i >> 1);
    /* The next line can be repeated any number of times to increase accuracy */
    u.x = u.x * (1.5f - xhalf * u.x * u.x);
    return u.x;
}

// Calculate attitude during runtime.
void calculate_attitude_nick(int sensor_data[]){
  /*--- Madgwick Filter ------------------------------------------------------*/
  float normalize;

  //Import and normalize accelerometer data
  float a_x = sensor_data[0];
  float a_y = sensor_data[1];
  float a_z = sensor_data[2];
  normalize = invSqrt(a_x*a_x + a_y*a_y + a_z*a_z);
  a_x *= normalize; a_y *= normalize; a_z *= normalize;

  // 1.09 = fudge factor. g_x in radians / sec
  float g_x = sensor_data[4] * (lsb_coefficient) * (1.0) * degrees_to_rad;
  float g_y = sensor_data[5] * (lsb_coefficient) * (1.0) * degrees_to_rad;
  float g_z = sensor_data[6] * (lsb_coefficient) * (1.0) * degrees_to_rad;

  // q_dot = 0.5 angular velocity rotation maxtrix * q.
  // Reference: A New Quaternion-Based Kalman Filter for Real-Time Attitude Estimation Using the Two-Step Geometrically-Intuitive Correction Algorithm. Equation 32 in section 2.3.1

  float qDot_0 = 0.5f*(-q_1*g_x - q_2*g_y - q_3*g_z);
  float qDot_1 = 0.5f*(q_0*g_x + q_2*g_z - q_3*g_y);
  float qDot_2 = 0.5f*(q_0*g_y + q_2*g_x - q_1*g_z);
  float qDot_3 = 0.5f*(q_0*g_z + q_1*g_y - q_2*g_x);

  /* References:
      1. https://nitinjsanket.github.io/tutorials/attitudeest/madgwick - (primary)
      2. Estimation of IMU and MARG orientation using a gradient descent algorithm (Sebastian O.H. Madgwick, Andrew J.L. Harrison, Ravi Vaidyanathan) - (supplementary) */

  // Setup for gradient descent algorithm: precalculate any values that occur more than once. Doing this saves the processer 30 multiplication operations.
  float q2_0 = q_0 * q_0; //a2
  float q2_1 = q_1 * q_1; //b2
  float q2_2 = q_2 * q_2; //c2
  float q2_3 = q_3 * q_3; //d2

  float _4q_0 = 4.0f * q_0; //4a
  float _4q_1 = 4.0f * q_1; //4b
  float _4q_2 = 4.0f * q_2; //4c
  float _4q_3 = 4.0f * q_3; //4d

  float _2q_0 = 2.0f * q_0; //2a
  float _2q_1 = 2.0f * q_1; //2b
  float _2q_2 = 2.0f * q_2; //2c
  float _2q_3 = 2.0f * q_3; //2d

  float _8q_1 = 8.0f * q_1; //8b
  float _8q_2 = 8.0f * q_2; //8c

  // Gradient Descent algorithm
  float delF_0 = _4q_0 * q2_2 + _4q_0 * q2_1 + _2q_2 * a_x - _2q_1 * a_y;
  float delF_1 = _8q_1*q2_1 + _4q_1*q2_3 + _4q_1*q2_0 - _4q_1 + _8q_1*q2_2 - _2q_3*a_x - _2q_0*a_y + _4q_1*a_z;
  float delF_2 = _8q_2*q2_2 - _4q_2 + _4q_2*q2_3 + _4q_2*q2_0 + _8q_2*q2_1 + _2q_0*a_x - _2q_3*a_y + _4q_2*a_z;
  float delF_3 = _4q_3*q2_2 + _4q_3*q2_1 - _2q_1*a_x - _2q_2*a_y;

  // Change correction_gain for more or less influence of accelerometer on gyro rates.
  qDot_0 -= correction_gain * delF_0;
  qDot_1 -= correction_gain * delF_1;
  qDot_2 -= correction_gain * delF_2;
  qDot_3 -= correction_gain * delF_3;
  q_0 += qDot_0 * sample_time;
  q_1 += qDot_1 * sample_time;
  q_2 += qDot_2 * sample_time;
  q_3 += qDot_3 * sample_time;

  normalize = invSqrt(q_0*q_0 + q_1*q_1 + q_2*q_2 + q_3*q_3);
  q_0 *= normalize; q_1 *= normalize; q_2 *= normalize; q_3 *= normalize;

  roll = atan2f(2*(q_0*q_1 + q_2*q_3), 1.0f - 2.0f*(q_1*q_1 + q_2*q_2)) * rad_to_degrees + 0.0f;
  pitch = asinf(2.0f * (q_0*q_2 - q_1*q_3)) * rad_to_degrees + 2.0f;
  yaw = atan2f(2*(q_0*q_3 + q_1*q_2), 1.0f - 2.0f*(q_2*q_2 + q_3*q_3)) * rad_to_degrees;

}


// Calculate attitude during runtime.
void calculate_attitude_eulerAngles(int sensor_data[]){
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
  //1.502 = coefficient that makes gyro attitue match accel attitude.
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


/*--- CALIBRATE IMU ----------------------------------------------------------*/
void calibrate_imu(){
  /* THE IMU MUST NOT BE MOVED DURING STARTUP */

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

    g_drift[0] += data_xyzt[4];
    g_drift[1] += data_xyzt[5];
    g_drift[2] += data_xyzt[6];

    accel_data_processing(&data_xyzt);

    free(data_xyzt); // Clear dynamic memory allocation

    delay(3);
  }
  // Find the averages drift / offset of the raw gyroscope data:
  g_drift[0] /= cal_count;
  g_drift[1] /= cal_count;
  g_drift[2] /= cal_count;
}

/*--- SETUP ------------------------------------------------------------------*/
void setup() {
  Serial.begin(2000000);
  Wire.begin();

  // Calibrate imu
  setup_mpu();
  calibrate_imu();
}

/*--- MAIN -------------------------------------------------------------------*/
void loop(){
  //sample_time = (micros() - elapsed_time) / 1000000.0f;
  elapsed_time = micros();

  //IMU
  int * data_xyzt;
  read_mpu(&data_xyzt);
  accel_data_processing(&data_xyzt);
  gyro_data_processing(&data_xyzt);

  time_1 = micros() - elapsed_time;

  if (imu_mode == 0){
    calculate_attitude_eulerAngles(data_xyzt);

    time_2 = micros() - elapsed_time;
    imu_mode = 1;
  } else if(imu_mode == 1){
    calculate_attitude_nick(data_xyzt);
    time_3 = micros() - elapsed_time;
    imu_mode = 0;
  }

  if (elapsed_time - last_time_print > 100000){
    Serial.print(time_1);
    Serial.print(" ");

    Serial.print(time_2);
    Serial.print(" ");
    Serial.print(time_3 - time_1);
    Serial.print("\n");

    last_time_print = micros();
  }

  //debug_loopTime();
  free(data_xyzt);  // Clear allocated memory for data array.

  // REFRESH RATE
  while (micros() - elapsed_time < 5000);
  // if (micros() - elapsed_time > 5500){  //Freeze if the loop takes too long
  //   while(true);
  // }
}
