#include <Wire.h>
#include <stdlib.h>
#include <Arduino.h>
#include <Servo.h>
/* Global Constants */
/*--- Propeller Servos -------------------------------------------------------*/
Servo right_prop;
double throttle = 1200;

//--- Simple Moving Average Globals ------------------------------------------*/
const int samples = 100;
int readings_x[samples];
int readings_y[samples];
int readings_z[samples];
long int read_index = 0;
long int total[3] = {0, 0, 0};
long int average[3] = {0, 0, 0};
int last_gyro_raw_x, last_gyro_raw_y, last_gyro_raw_z;

/*--- IMU Globals ------------------------------------------------------------*/
// NOTE:
/* Most of these can be made private, update when you have time. */
float acceleration_total_vector, acceleration_angle_roll,
      acceleration_angle_pitch, acceleration_angle_yaw;
// Delete the following raw variables on completetion of IMU.
float acceleration_raw_vector, acceleration_raw_roll,
      acceleration_raw_pitch;
float angle_gyro_roll, angle_gyro_pitch;
float rad_to_degrees = 57.296;
float degrees_to_rad = 0.01745556;
float time_coeficcient = 0.00007634;
float angle_roll, angle_pitch, initial_roll = 0, initial_pitch = 0;
// Initial Value Globals:
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
bool startup = true;

/*--- Time Control -----------------------------------------------------------*/
int refresh_rate = 200;
const float loop_micros = (1 / (float)refresh_rate) * 1000000;
float previous_time, time, loop_time;

/*--- PID Globals ------------------------------------------------------------*/

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
  //        0x08 = Range of the gyro in degree/sec (data sheet)
  //        0x10 = 1000 degree / sec range
  // 0x18 = 2000 degree / sec range
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x18);
  Wire.endTransmission();
}

/*--- read accelerometer -----------------------------------------------------*/
void read_mpu(int ** sensor_output_array){
  /*Set up a pointer for our array: Has to be passed in
  from outside of the function. Memory must be dynamically
  assigned: */
  int array_size = 7;
  *sensor_output_array = malloc(sizeof(int) * array_size);
  /* Access the accellerometer register and requst
  14 bits. Assign each high and low bit to a variable. */
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 14);
  // Wait until all of the bits have been recieved.
  while(Wire.available() < 14){};
  // Assign values to each element of the array:
  (*sensor_output_array)[0] = Wire.read()<<8|Wire.read(); // a_x
  (*sensor_output_array)[1] = Wire.read()<<8|Wire.read(); // a_y
  (*sensor_output_array)[2] = Wire.read()<<8|Wire.read(); // a_z
  (*sensor_output_array)[3] = Wire.read()<<8|Wire.read(); // temp
  (*sensor_output_array)[4] = Wire.read()<<8|Wire.read(); // g_x
  (*sensor_output_array)[5] = Wire.read()<<8|Wire.read(); // g_y
  (*sensor_output_array)[6] = Wire.read()<<8|Wire.read(); // g_z
  // No need to return anything.
}

/*--- Moving Average FIlter --------------------------------------------------*/
void data_processing_startup(int * sensor_data[]){
  total[0] -= readings_x[read_index];
  total[1] -= readings_y[read_index];
  total[2] -= readings_z[read_index];
  readings_x[read_index] = (*sensor_data)[0];
  readings_y[read_index] = (*sensor_data)[1];
  readings_z[read_index] = (*sensor_data)[2];
  total[0] += readings_x[read_index];
  total[1] += readings_y[read_index];
  total[2] += readings_z[read_index];
  read_index += 1;
  if (read_index >= samples){
    read_index = 0;
  }
  average[0] = total[0] / samples;
  average[1] = total[1] / samples;
  average[2] = total[2] / samples;
  /*NOTE: Merge this function with data_processing when you have time. */
}

/*--- Moving Average FIlter --------------------------------------------------*/
void data_processing(int * sensor_data[]){
  total[0] -= readings_x[read_index];
  total[1] -= readings_y[read_index];
  total[2] -= readings_z[read_index];
  readings_x[read_index] = (*sensor_data)[0];
  readings_y[read_index] = (*sensor_data)[1];
  readings_z[read_index] = (*sensor_data)[2];
  total[0] += readings_x[read_index];
  total[1] += readings_y[read_index];
  total[2] += readings_z[read_index];
  read_index += 1;
  if (read_index >= samples){
    read_index = 0;
  }
  average[0] = total[0] / samples;
  average[1] = total[1] / samples;
  average[2] = total[2] / samples;

  /* Process gyroscope data: */
  (*sensor_data)[4] -= gyro_x_cal;
  (*sensor_data)[5] -= gyro_y_cal;
  (*sensor_data)[6] -= gyro_z_cal;
}

/*--- Calculate Roll Pitch & Yaw ---------------------------------------------*/
void calculate_attitude(int sensor_data[]){
  /*--- Attitude from accelerometer -------------*/
  /* To determine angle we are on based off of acceleration
  data, first we calculate the total acceleration from gravity
  vector: */
  acceleration_total_vector =
  sqrt(pow(average[0], 2) + pow(average[1], 2) + pow(average[2], 2));
  /*--- X-Roll Angle ---*/
  acceleration_angle_roll =
  asin((float)average[1] / acceleration_total_vector) * rad_to_degrees;
  /*--- Y-Pitch Angle ---*/
  acceleration_angle_pitch =
  asin((float)average[0] / acceleration_total_vector) * rad_to_degrees;

  /*--- Correct initial angle errors ---*/
  acceleration_angle_roll += 0;
  acceleration_angle_pitch += 0;

  /*--- Angle from Gyroscope -------------------------------------------------*/
  /* As the data giver from the gryoscope is angular velocity (degrees / sec) we
   must multiply the output by the time elapsed in each loop. This time is
   calculated each loop. (see data sheet & registry map). */
  // 32.8 ~ 1000 Degree / Sec Modifier
  // 65.5 ~ 500 Degrees / Sec Modifier
  // 131.0 ~ 250 Degrees / Sec Modifier
  angle_gyro_roll = ((float)sensor_data[4] / 32.8) * loop_time;
  angle_gyro_pitch = ((float)sensor_data[5] / 32.8) * loop_time;
  /* The yaw of the gyro must be used to transfer
  the roll angle to pitch and vice versa.  */
  angle_gyro_roll -=
  angle_gyro_pitch * sin((sensor_data[6] / 32.8) * loop_time * degrees_to_rad);
  angle_gyro_pitch +=
  angle_gyro_roll * sin((sensor_data[6] / 32.8) * loop_time * degrees_to_rad);

  /*--- Complimentary FIlter --------------------*/
  /* Combine the sensor outputs & add the last gyro angle to the current one. */
  angle_roll =
  0.8 * (angle_roll + angle_gyro_roll) + (-1) * 0.2 * acceleration_angle_roll;
  angle_pitch =
  0.8 * (angle_pitch + angle_gyro_pitch) + 0.2 * acceleration_angle_pitch;
}

/*--- Calibrate IMU ----------------------------------------------------------*/
void calibrate_imu(){
  /* THE IMU MUST NOT BE MOVED DURING SETUP */
  /*--- Simple Moving Average Setup ---*/
  for (int i = 0; i < samples; i++){
    readings_x[i] = 0;
    readings_y[i] = 0;
    readings_z[i] = 0;
  }

  /*--- Calibrate gyroscope data and initial attitude. ---*/
  Serial.print("\nCalibrating \n");
  for (int i = 0; i < 2000; i ++){
    if(i % 125 == 0){
      /* Print the loading bar blips 16 times */
      Serial.print("*");
    }
    // Collect data from MPU
    int * data_xyzt;
    read_mpu(&data_xyzt);
    gyro_x_cal += data_xyzt[4];
    gyro_y_cal += data_xyzt[5];
    gyro_z_cal += data_xyzt[6];
    data_processing_startup(&data_xyzt);
    calculate_attitude(data_xyzt);
    initial_roll += acceleration_angle_roll;
    initial_pitch += acceleration_angle_pitch;
    free(data_xyzt); // Clear dynamic memory allocation
    delay(3);
  }
  /* Find the average value of the data that was recorded above: */
  gyro_x_cal /= 2000;
  gyro_y_cal /= 2000;
  gyro_z_cal /= 2000;
  initial_roll /= 2000;
  initial_pitch /= 2000;

  /*--- Set Initial Angles ---*/
  angle_gyro_roll = (-1) * initial_roll;
  angle_gyro_pitch = initial_pitch;
}

/*--- Flight Controller ------------------------------------------------------*/

/*--- Loop Time --------------------------------------------------------------*/

/*--- Debugging --------------------------------------------------------------*/
void debugging(){

  // Serial.println(0);
  // Serial.print(" ");
  // Serial.print(loop_time);
  // Serial.print(" ");
  // Serial.print(loop_micros);
  // Serial.print(" ");

  /*--- Print ---*/
  // Serial.print(90);
  // Serial.print(" ");
  // Serial.print(-90);
  // Serial.print(" ");

  // Gyroscope output:
  Serial.print("\n");
  Serial.print(angle_roll);
  Serial.print(" ");
  Serial.print(angle_pitch);
}

/*--- Setup ------------------------------------------------------------------*/
void setup() {
  Serial.begin(115200);
  Wire.begin();
  /*--- Propellers ---*/
  right_prop.attach(5);
  right_prop.writeMicroseconds(1000);
  /*--- Calibration ---*/
  time = millis();
  setup_mpu();
  calibrate_imu();
  /* Specify the desired loop time based on refresh rate */
}

/*--- Main -------------------------------------------------------------------*/
void loop() {
  /* Time taken in the last cycle: */
  previous_time = time;
  time = millis();
  loop_time = (time - previous_time) / 1000;
  long int elapsed_time = micros();

  /*--- IMU ---*/
  int * data_xyzt;
  read_mpu(&data_xyzt);
  data_processing(&data_xyzt);
  calculate_attitude(data_xyzt);
  free(data_xyzt);  // Clear allocated memory for array.

  /*--- Flight Controller ---*/
  right_prop.writeMicroseconds(throttle);

  /*--- Debugging & Printing ---*/
  debugging();

  /*--- Time Control ---*/
  long int loop_time = micros() - elapsed_time;
  if (loop_time > loop_micros) {
      Serial.print("Loop time too long! ");
      delay(3000);
  }

  while ((micros() - elapsed_time) < loop_micros) {};
}
