#include <Wire.h>
#include <stdlib.h>
#include <Servo.h>
#include <IRremote.h>

#define ACTIVATED HIGH
unsigned long elapsed_time;
float sample_time;
unsigned long last_time_print;

//--- Simple Moving Average Globals ------------------------------------------*/
const int sma_samples = 15;
int a_x_readings[sma_samples];
int a_y_readings[sma_samples];
int a_z_readings[sma_samples];
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
float correction_gain = 0.2f;

/*--- Debugging --------------------------------------------------------------*/
void debugging(){
  int mode = 1;

  // Serial Print has a significant impact on performance. Only use it once every n scans.
  if (elapsed_time - last_time_print > 20000){
    if (elapsed_time - last_time_print > 20000){
      if(mode == 1){
        Serial.print("Roll: ");
        Serial.print(roll);

        Serial.print(" - Pitch: ");
        Serial.print(pitch);

        Serial.print(" - Pitch: ");
        Serial.print(yaw);

        Serial.print("\n");
      }

    last_time_print = micros();
    }

    if (elapsed_time - last_time_print > 20000){
      last_time_print = micros();
    }
  }
}

// I don't have an oscilloscope so here you go:
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
// Simple moving average filter. This method smoothes out noisey accelerometer data. It isn't too expensive. Be careful when setting the number of sma_samples: A large sma_samples will lead to a large time-delay, too few sma_samples will lead to a negligible smoothing effect.
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
  if (a_read_index >= sma_samples){
    a_read_index = 0;
  }
  a_read_ave[0] = a_read_total[0] / sma_samples;
  a_read_ave[1] = a_read_total[1] / sma_samples;
  a_read_ave[2] = a_read_total[2] / sma_samples;
}

// Remove the average gyroscope drift / offset (recorded in the calibration method) from the gyroscope data that is recorded during each scan.
void gyro_data_processing(int * sensor_data[]){
  (*sensor_data)[4] -= g_drift[0];
  (*sensor_data)[5] -= g_drift[1];
  (*sensor_data)[6] -= g_drift[2];
}

/*--- CALCULATE ATTITUDE -----------------------------------------------------*/
// Cheapest/fastest inverse square root I could find (99.94% accurate to 1 / sqrt(x))
// Source: http://www.dbfinteractive.com/forum/index.php?topic=6269.0
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
void calculate_attitude(int sensor_data[]){
  /*--- Madgwick Filter ------------------------------------------------------*/
  float normalize;

  //Import and normalize accelerometer data
  float a_x = sensor_data[0];
  float a_y = sensor_data[1];
  float a_z = sensor_data[2];

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

  // Calculate orientation from accelerometer data
  // Reference: Keeping a Good Attitude: A Quaternion-Based Orientation Filter for IMUs and MARGs
  // Equation 25

  float qa_0, qa_1, qa_2, qa_3;

  if(a_z >= 0){
    qa_0 = sqrt((a_z + 1.0f) / (2.0f));
    qa_1 = (-1.0f) * (a_y) * invSqrt(2.0f * (a_z * 1.0f));
    qa_2 = a_x * invSqrt(2 * (a_z + 1.0f));
    qa_3 = 0.0f;
  }
  else if (a_z < 0){
    qa_0 = (-1.0f) * a_y * invSqrt(2.0f * (1.0f - a_z));
    qa_1 = sqrt((1.0f - a_z) / (2.0f));
    qa_2 = 0.0f;
    qa_3 = a_x * invSqrt(2 * (1.0f - a_z));
  }

  // Complimentary Filter
  q_0 += qDot_0 * sample_time * 0.9f + qa_0 * 0.1f;
  q_1 += qDot_1 * sample_time * 0.9f + qa_1 * 0.1f;
  q_2 += qDot_2 * sample_time * 0.9f + qa_2 * 0.1f;
  q_3 += qDot_3 * sample_time * 0.9f + qa_3 * 0.1f;

  // Normalize orientation quaternion to account for noise
  normalize = invSqrt(q_0*q_0 + q_1*q_1 + q_2*q_2 + q_3*q_3);
  q_0 *= normalize; q_1 *= normalize; q_2 *= normalize; q_3 *= normalize;

  // Convert back into Euler angles
  roll = atan2f(2*(q_0*q_1 + q_2*q_3), 1.0f - 2.0f*(q_1*q_1 + q_2*q_2)) * rad_to_degrees + 0.0f;
  pitch = asinf(2.0f * (q_0*q_2 - q_1*q_3)) * rad_to_degrees + 2.0f;
  yaw = atan2f(2*(q_0*q_3 + q_1*q_2), 1.0f - 2.0f*(q_2*q_2 + q_3*q_3)) * rad_to_degrees;
}

/*--- CALIBRATE IMU ----------------------------------------------------------*/
void calibrate_imu(){
  /* KEEP IMU STATIONARY DURING STARTUP */

  /*--- Simple Moving Average Setup ---*/
  for (int i = 0; i < sma_samples; i++){
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

    free(data_xyzt); //Clear memory

    delay(3);
  }
  // Average drift / offset of the raw gyroscope data:
  g_drift[0] /= cal_count;
  g_drift[1] /= cal_count;
  g_drift[2] /= cal_count;
}

/*--- SETUP ------------------------------------------------------------------*/
void setup() {
  pinMode(7, INPUT);
  Serial.begin(2000000);
  Wire.begin();

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
  read_mpu(&data_xyzt);
  accel_data_processing(&data_xyzt);
  gyro_data_processing(&data_xyzt);
  calculate_attitude(data_xyzt);
  //debug_loopTime();
  free(data_xyzt);  // Clear allocated memory for data array.

  // DEBUGGING
  //debugging();
  debug_loopTime();

  // REFRESH RATE
  while (micros() - elapsed_time < 5500);
  // if (micros() - elapsed_time > 5500){  //Freeze if the loop takes too long
  //   while(true);
  // }
}
