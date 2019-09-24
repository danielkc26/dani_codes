
#include "encoding.h"
#include <SoftwareSerial32.h> //SW_UART eshtablishment
#include <SlowSoftI2CMaster.h>//SW_I2C eshtablishment
#include <SlowSoftWire.h>


SlowSoftWire Wire = SlowSoftWire(18, 19, true);
SlowSoftWire EEPROM = SlowSoftWire(2, 3, true);
SoftwareSerial32 ss(15, 16); // RX,TX

#define EEPROM_I2C_ADDRESS 0x50
#define ADC_CHANNEL        1
//#define BAT_LOW_LED        1
#define GPS_LED            8
#define STATUS_LED         9
#include <Adafruit_MCP3008.h>

Adafruit_MCP3008 adc;

int ppm_pin = 17;
int ppm_pinmask;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_roll = 0.75; //1.875;       //Gain setting for the roll P-controller
float pid_i_gain_roll = 0.00010;           //Gain setting for the roll I-controller
float pid_d_gain_roll = 3.5;               //Gain setting for the roll D-controller

int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

//float pid_p_gain_yaw = 4.0;                //Gain setting for the pitch P-controller. //4.0
float pid_p_gain_yaw = 3.5;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)

float gps_p_gain = 0.2;                    //Gain setting for the GPS P-controller (default = 2.7).
float gps_d_gain = 0.5;                    //Gain setting for the GPS D-controller (default = 6.5).

boolean auto_level = true;                 //Auto level on (true) or off (false)

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
byte eeprom_data[36];
byte highByte, lowByte;
uint8_t channel_select_counter;
int32_t measured_time, measured_time_start;
volatile int receiver_input_channel_1,
         receiver_input_channel_2,
         receiver_input_channel_3,
         receiver_input_channel_4,
         receiver_input_channel_5,
         receiver_input_channel_6, pid_roll_setpoint_base, pid_pitch_setpoint_base;
int esc_1, esc_2, esc_3, esc_4;
int throttle, battery_voltage;
int cal_int, start, gyro_address;
int receiver_input[7];
int16_t temperature;
int16_t acc_axis[4], gyro_axis[4];
float roll_level_adjust, pitch_level_adjust;

int16_t acc_x, acc_y, acc_z;
int acc_total_vector;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer, esc_loop_timer1, esc_loop_timer2, esc_loop_timer3;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
unsigned long loop_timer, loop_timer1, loop_timer2, loop_timer3, loop_timer4, loop_counter;
int gyro_pitch, gyro_roll, gyro_yaw;
int gyro_axis_cal[4];
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll, angle_yaw;
boolean gyro_angles_set;

uint32_t pin4 = 4; uint32_t pin5 = 5; uint32_t pin6 = 6; uint32_t pin7 = 7;
uint32_t bitmask4 = digitalPinToBitMask(pin4);
uint32_t bitmask5 = digitalPinToBitMask(pin5);
uint32_t bitmask6 = digitalPinToBitMask(pin6);
uint32_t bitmask7 = digitalPinToBitMask(pin7);

//GPS variables
uint8_t read_serial_byte, incomming_message[250], number_used_sats, fix_type;
uint8_t waypoint_set, latitude_north, longiude_east ;
uint16_t message_counter;
int16_t gps_add_counter;
int32_t l_lat_gps, l_lon_gps, lat_gps_previous, lon_gps_previous;
int32_t lat_gps_actual, lon_gps_actual, l_lat_waypoint, l_lon_waypoint;
float gps_pitch_adjust_north, gps_pitch_adjust, gps_roll_adjust_north, gps_roll_adjust;
float lat_gps_loop_add, lon_gps_loop_add, lat_gps_add, lon_gps_add;
uint8_t new_line_found, new_gps_data_available, new_gps_data_counter;
uint8_t gps_rotating_mem_location;
int32_t gps_lat_total_avarage, gps_lon_total_avarage;
int32_t gps_lat_rotating_mem[40], gps_lon_rotating_mem[40];
int32_t gps_lat_error, gps_lon_error;
int32_t gps_lat_error_previous, gps_lon_error_previous;
uint32_t gps_watchdog_timer;

uint8_t flight_mode, flight_mode_counter, flight_mode_led;

int channel = ADC_CHANNEL;
int lowExit = 0;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup routine
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(57600);

  pinMode(STATUS_LED, OUTPUT);
  pinMode(GPS_LED, OUTPUT);
  //  pinMode(BAT_LOW_LED, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  delay(2000);
  Wire.setClock(400000L);
  Wire.begin();
  delay(250);

  EEPROM.begin();
  EEPROM.setClock(400000L);                                           //Start the wire library as master
  delay(250);

  
  delay(550);
  // Software SPI (specify all, use any available digital)
  // (sck, mosi, miso, cs);
  //adc.begin(15, 17, 16, 13); // Assign pins for ADC
  // adc.begin(13, 11, 12, 10);

  // Hardware SPI
  adc.begin();//uses (13, 11, 12, 10-cs);
  delay(500);


  Serial.println(">>START");
  //Copy the EEPROM data for fast access data.
  for (start = 0; start <= 35; start++)eeprom_data[start] = EEPROMreadAddr(start);
  start = 0;                                                                //Set start back to zero.
  gyro_address = eeprom_data[32];                                           //Store the gyro address in the variable.

  //GPS setup
 
  //Configure digital pins.

  //Use the led on the Arduino for startup indication.
  //digitalWrite(STATUS_LED, LOW);                                                   //Turn on the warning led.

  pinMode(ppm_pin, INPUT_PULLUP);
  ppm_pinmask = digitalPinToBitMask(ppm_pin);                                      //Setup interrupt
  attachInterrupt(digitalPinToInterrupt(ppm_pin), ISR, RISING);

  set_csr(mstatus, MSTATUS_MIE);
    //Check the EEPROM signature to make sure that the setup program is executed.
    while(eeprom_data[33] != 'J' || eeprom_data[34] != 'M' || eeprom_data[35] != 'B')delay(10);
    Serial.println(">>Verified-EEPROM");
    //The flight controller needs the MPU-6050 with gyro and accelerometer
    //If setup is completed without MPU-6050 stop the flight controller program
    if(eeprom_data[31] == 2 || eeprom_data[31] == 3)delay(10);

    set_gyro_registers();                                                     //Set the specific gyro registers.
    Serial.println(">>START-GYRO");
    for (cal_int = 0; cal_int < 1250 ; cal_int ++){                           //Wait 5 seconds before continuing.

    //Set digital port 4, 5, 6 and 7 high.
    GPIO_REG(GPIO_OUTPUT_VAL) |= bitmask4 | bitmask5 | bitmask6| bitmask7 ;//HIGH
    delayMicroseconds(1000);                                                //Wait 1000us.

     //Set digital port 4, 5, 6 and 7 low.
    GPIO_REG(GPIO_OUTPUT_VAL) &= ~bitmask4 & (~bitmask5) & (~bitmask6) & (~bitmask7);//LOW
    delayMicroseconds(3000);                                                //Wait 3000us.
    }
    Serial.println(">>Gyro Calib...");
    //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
    for (cal_int = 0; cal_int < 2000 ; cal_int ++){                         //Take 2000 readings for calibration.
    digitalWrite(STATUS_LED, LOW);
    if(cal_int % 15 == 0)digitalWrite(STATUS_LED,HIGH);                //Change the led status to indicate calibration.
    gyro_signalen();                                                        //Read the gyro output.
    gyro_axis_cal[1] += gyro_axis[1];                                       //Ad roll value to gyro_roll_cal.
    gyro_axis_cal[2] += gyro_axis[2];                                       //Ad pitch value to gyro_pitch_cal.
    gyro_axis_cal[3] += gyro_axis[3];                                       //Ad yaw value to gyro_yaw_cal.

    //We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while calibrating the gyro.
    //Set digital port 4, 5, 6 and 7 high.
    GPIO_REG(GPIO_OUTPUT_VAL) |= bitmask4 | bitmask5 | bitmask6| bitmask7 ; //HIGH
    delayMicroseconds(1000);                                                //Wait 1000us.

    GPIO_REG(GPIO_OUTPUT_VAL) &= ~bitmask4 & (~bitmask5) & (~bitmask6) & (~bitmask7);//LOW
    delay(3);                                                               //Wait 3 milliseconds before the next loop.
    }
    Serial.println(">>Calibrated.");

    //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
    gyro_axis_cal[1] /= 2000;                                               //Divide the roll total by 2000.
    gyro_axis_cal[2] /= 2000;                                               //Divide the pitch total by 2000.
    gyro_axis_cal[3] /= 2000;                                               //Divide the yaw total by 2000.
  
 
  Serial.println(">>ReceiverStart..."); 
  //Wait until the receiver is active and the throtle is set to the lower position.
  while (receiver_input_channel_3 < 990 || receiver_input_channel_3 > 1020 || receiver_input_channel_4 < 1400) {
    receiver_input_channel_3 = convert_receiver_channel(3);                 //Convert the actual receiver signals for throttle to the standard 1000 - 2000us
    receiver_input_channel_4 = convert_receiver_channel(4);                 //Convert the actual receiver signals for yaw to the standard 1000 - 2000us
    start ++;                                                               //While waiting increment start whith every loop.

    //We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while waiting for the receiver inputs.
  
    //Set digital poort 4, 5, 6 and 7 high.
    GPIO_REG(GPIO_OUTPUT_VAL) |= bitmask4 | bitmask5 | bitmask6 | bitmask7 ; //HIGH
    delayMicroseconds(1000);                                                //Wait 1000us.

    //Set digital poort 4, 5, 6 and 7 low.
    GPIO_REG(GPIO_OUTPUT_VAL) &= ~bitmask4 & (~bitmask5) & (~bitmask6) & (~bitmask7);//LOW
    delay(3);                                                               //Wait 3 milliseconds before the next loop.
    digitalWrite(STATUS_LED, HIGH);
    if (start == 125) {                                                     //Every 125 loops (500ms).
     digitalWrite(STATUS_LED, LOW);                                   //Change the led status.
      start = 0;                                                            //Start again at 0.
    }
  }
  start = 0;                                                                //Set start back to 0.
  
  Serial.println(">>Setup Done.");
  gps_setup();
  delay(250);
  //HIFIVE1 does not have analog inputs , hence we use an ADC. Here MCP3008 is used
  //Load the battery voltage to the battery_voltage variable.
  //65 is the voltage compensation for the diode.
  //12.6V equals ~5V @ Analog 0.
  //12.6V equals 1023 analogRead(0).
  //1260 / 1023 = 1.2317.
  //The variable battery_voltage holds 1050 if the battery voltage is 10.5V.
  //  battery_voltage = (analogRead(0) + 65) * 1.2317;
  // Serial.print("adc-");Serial.println(adc.readADC(channel));

  battery_voltage = (adc.readADC(channel) + 65) * 1.2317; // no diode is used

  loop_timer = micros();                                                    //Set the timer for the next loop.
  //When everything is done, turn off the led.
//  digitalWrite(STATUS_LED, HIGH); //Turn off the warning led.

}
int roll=0;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {

  loop_timer = micros();
  flight_mode = 0;                                                                 //In all other situations the flight mode is 1;
  if (receiver_input_channel_5 >= 1500)flight_mode = 1;                       //If channel 6 is between 1200us and 1600us the flight mode is 2
  // if (channel_5 >= 1600 && channel_5 < 2100)flight_mode = 3;                       //If channel 6 is between 1600us and 1900us the flight mode is 3

  gyro_signalen();

 
  if (gps_add_counter >= 0)gps_add_counter --;
  read_gps();
 

  //65.5 = 1 deg/sec (check the datasheet of the MPU-6050 for more information).
  gyro_roll_input = (gyro_roll_input * 0.7) + (((float)gyro_roll / 65.5) * 0.3);   //Gyro pid input is deg/sec.
  gyro_pitch_input = (gyro_pitch_input * 0.7) + (((float)gyro_pitch / 65.5) * 0.3);//Gyro pid input is deg/sec.
  gyro_yaw_input = (gyro_yaw_input * 0.7) + (((float)gyro_yaw / 65.5) * 0.3);      //Gyro pid input is deg/sec.

  //Gyro angle calculations
  //for 250HZ
  //0.0000611 = 1 / 250Hz / 65.5
  angle_pitch += (float)gyro_pitch * 0.0000611;                                    //Calculate the traveled pitch angle and add this to the angle_pitch variable.
  angle_roll += (float)gyro_roll * 0.0000611;                                      //Calculate the traveled roll angle and add this to the angle_roll variable.
  angle_yaw += (float)gyro_yaw * 0.0000611;                                        //Calculate the traveled yaw angle and add this to the angle_yaw variable.
  if (angle_yaw < 0) angle_yaw += 360;                                             //If the compass heading becomes smaller then 0, 360 is added to keep it in the 0 till 360 degrees range.
  else if (angle_yaw >= 360) angle_yaw -= 360;                                     //If the compass heading becomes larger then 360, 360 is subtracted to keep it in the 0 till 360 degrees range.

  //for 250HZ
  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch -= angle_roll * sin((float)gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the roll angle to the pitch angel.
  angle_roll += angle_pitch * sin((float)gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the pitch angle to the roll angel.


  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z)); //Calculate the total accelerometer vector.

  if (abs(acc_y) < acc_total_vector) {                                      //Prevent the asin function to produce a NaN
    angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;       //Calculate the pitch angle.
  }
  if (abs(acc_x) < acc_total_vector) {                                      //Prevent the asin function to produce a NaN
    angle_roll_acc = asin((float)acc_x / acc_total_vector) * -57.296;       //Calculate the roll angle.
  }

  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration.
  angle_pitch_acc -= 0.0;                                                   //Accelerometer calibration value for pitch.
  angle_roll_acc -= 0.00;                                                    //Accelerometer calibration value for roll.


  angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;            //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
  angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;               //Correct the drift of the gyro roll angle with the accelerometer roll angle.


  pitch_level_adjust = angle_pitch * 15;                                    //Calculate the pitch angle correction
  roll_level_adjust = angle_roll * 15;                                      //Calculate the roll angle correction

  if (!auto_level) {                                                        //If the quadcopter is not in auto-level mode
    pitch_level_adjust = 0;                                                 //Set the pitch angle correction to zero.
    roll_level_adjust = 0;                                                  //Set the roll angle correcion to zero.
  }

  pid_roll_setpoint_base = receiver_input_channel_1;                                              //Normally channel_1 is the pid_roll_setpoint input.
  pid_pitch_setpoint_base = receiver_input_channel_2;                                             //Normally channel_2 is the pid_pitch_setpoint input.


  if (flight_mode == 1 && waypoint_set == 1) {
    pid_roll_setpoint_base += gps_roll_adjust;
    pid_pitch_setpoint_base += gps_pitch_adjust;
  }

  //Because we added the GPS adjust values we need to make sure that the control limits are not exceded.
  if (pid_roll_setpoint_base > 2000)pid_roll_setpoint_base = 2000;
  if (pid_roll_setpoint_base < 1000)pid_roll_setpoint_base = 1000;
  if (pid_pitch_setpoint_base > 2000)pid_pitch_setpoint_base = 2000;
  if (pid_pitch_setpoint_base < 1000)pid_pitch_setpoint_base = 1000;


  //The PID set point in degrees per second is determined by the roll receiver input.
  //In the case of deviding by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_roll_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if (receiver_input_channel_1 > 1508)pid_roll_setpoint = pid_roll_setpoint_base - 1508;
  else if (receiver_input_channel_1 < 1492)pid_roll_setpoint = pid_roll_setpoint_base - 1492;

  pid_roll_setpoint -= roll_level_adjust;                                   //Subtract the angle correction from the standardized receiver roll input value.
  pid_roll_setpoint /= 3.0;                                                 //Divide the setpoint for the PID roll controller by 3 to get angles in degrees.


  //The PID set point in degrees per second is determined by the pitch receiver input.
  //In the case of deviding by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_pitch_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if (receiver_input_channel_2 > 1508)pid_pitch_setpoint = pid_pitch_setpoint_base - 1508;
  else if (receiver_input_channel_2 < 1492)pid_pitch_setpoint = pid_pitch_setpoint_base - 1492;

  pid_pitch_setpoint -= pitch_level_adjust;                                  //Subtract the angle correction from the standardized receiver pitch input value.
  pid_pitch_setpoint /= 3.0;                                                 //Divide the setpoint for the PID pitch controller by 3 to get angles in degrees.

  //The PID set point in degrees per second is determined by the yaw receiver input.
  //In the case of deviding by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_yaw_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if (receiver_input_channel_3 > 1050) { //Do not yaw when turning off the motors.
    if (receiver_input_channel_4 > 1508)pid_yaw_setpoint = (receiver_input_channel_4 - 1508) / 3.0;
    else if (receiver_input_channel_4 < 1492)pid_yaw_setpoint = (receiver_input_channel_4 - 1492) / 3.0;
  }

  calculate_pid();                                                       //PID inputs are known. So we can calculate the pid output.

  //For starting the motors: throttle low and yaw left (step 1).
  if (receiver_input_channel_3 < 1050 && receiver_input_channel_4 < 1050)start = 1;

  //When yaw stick is back in the center position start the motors (step 2).
  if (start == 1 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1450) {
    start = 2;

    angle_pitch = angle_pitch_acc;                                          //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
    angle_roll = angle_roll_acc;                                            //Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started.
    gyro_angles_set = true;                                                 //Set the IMU started flag.

    //Reset the PID controllers for a bumpless start.
    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_i_mem_yaw = 0;
    pid_last_yaw_d_error = 0;
  }
  //Stopping the motors: throttle low and yaw right.
  if (start == 2 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1950)start = 0;


  //The battery voltage is needed for compensation.
  //A complementary filter is used to reduce noise.
  //0.09853 = 0.08 * 1.2317.
  
  battery_voltage = battery_voltage * 0.92 + (adc.readADC(channel) + 65) * 0.09853;
  //Turn on the led if battery voltage is to low.
  //Serial.println(battery_voltage);
  if (battery_voltage < 1000 && battery_voltage > 600);//digitalWrite(STATUS_LED, LOW);

  throttle = receiver_input_channel_3;                                      //We need the throttle signal as a base signal.

  if (start == 2) {

    if (throttle > 1800) throttle = 1800;                                   //We need some room to keep full control at full throttle.
    esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
    esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
    esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
    esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)

      if (battery_voltage < 1240 && battery_voltage > 800){                   //Is the battery connected?

        esc_1 += esc_1 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-1 pulse for voltage drop.
        esc_2 += esc_2 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-2 pulse for voltage drop.
        esc_3 += esc_3 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-3 pulse for voltage drop.
        esc_4 += esc_4 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-4 pulse for voltage drop.

      } 

    if (esc_1 < 1100) esc_1 = 1200;                                         //Keep the motors running.
    if (esc_2 < 1100) esc_2 = 1200;                                         //Keep the motors running.
    if (esc_3 < 1100) esc_3 = 1200;                                         //Keep the motors running.
    if (esc_4 < 1100) esc_4 = 1200;                                         //Keep the motors running.

    if (esc_1 > 2000)esc_1 = 2000;                                          //Limit the esc-1 pulse to 2000us.
    if (esc_2 > 2000)esc_2 = 2000;                                          //Limit the esc-2 pulse to 2000us.
    if (esc_3 > 2000)esc_3 = 2000;                                          //Limit the esc-3 pulse to 2000us.
    if (esc_4 > 2000)esc_4 = 2000;                                          //Limit the esc-4 pulse to 2000us.
  }
  else {

    esc_1 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-1.
    esc_2 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-2.
    esc_3 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-3.
    esc_4 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-4.
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////
  //Creating the pulses for the ESC's is explained in this video:
  //https://youtu.be/fqEkVcqxtU8
  ////////////////////////////////////////////////////////////////////////////////////////////////////

  //! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !
  //Because of the angle calculation the loop time is getting very important. If the loop time is
  //longer or shorter than 4000us the angle calculation is off. If you modify the code make sure
  //that the loop time is still 4000us and no longer! More information can be found on
  //the Q&A page:
  //! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !

  // loop_timer = micros();               //Set the timer for the next loop.
  //All the information for controlling the motor's is available.
  //The refresh rate is 250Hz. That means the esc's need there pulse every 4ms.
  //if (micros() - loop_timer > 4050)digitalWrite(STATUS_LED, LOW);                  //Turn on the LED if the loop time exceeds 4050us.
  while (micros() - loop_timer < 4000);                                     //We wait until 4000us are passed.
  lowExit = 0;
  loop_timer = micros();

  GPIO_REG(GPIO_OUTPUT_VAL) |= bitmask4 | bitmask5 | bitmask6 | bitmask7 ; //HIGH   //Set digital outputs 4,5,6 and 7 high.
  timer_channel_1 = esc_1 + loop_timer;                                     //Calculate the time of the faling edge of the esc-1 pulse.
  timer_channel_2 = esc_2 + loop_timer;                                     //Calculate the time of the faling edge of the esc-2 pulse.
  timer_channel_3 = esc_3 + loop_timer;                                     //Calculate the time of the faling edge of the esc-3 pulse.
  timer_channel_4 = esc_4 + loop_timer;                                     //Calculate the time of the faling edge of the esc-4 pulse.


  //There is always 1000us of spare time. So let's do something usefull that is very time consuming.
  //Get the current gyro and receiver data and scale it to degrees per second for the pid calculations.

  // while(PORTD >= 16){
  while (lowExit < 1) { //Stay in this loop until output 4,5,6 and 7 are low.
    esc_loop_timer = micros();                                         //Read the current time.
    if (timer_channel_1 <= esc_loop_timer) {
      GPIO_REG(GPIO_OUTPUT_VAL) &= ~bitmask4 ; //Set digital output 4 to low if the time is expired.
    }//digitalWrite(4,LOW);}

    if (timer_channel_2 <= esc_loop_timer ) {
      GPIO_REG(GPIO_OUTPUT_VAL) &= ~bitmask5; //Set digital output 5 to low if the time is expired.
    }//digitalWrite(5,LOW);}

    if (timer_channel_3 <= esc_loop_timer  ) {
      GPIO_REG(GPIO_OUTPUT_VAL) &= ~bitmask6; //Set digital output 6 to low if the time is expired.
    }// digitalWrite(6,LOW);}

    if (timer_channel_4 <= esc_loop_timer ) {
      GPIO_REG(GPIO_OUTPUT_VAL) &= ~bitmask7; //Set digital output 7 to low if the time is expired.
    }//digitalWrite(7,LOW);lowExit=1;}

    if ( (timer_channel_1 <= esc_loop_timer) &&
         (timer_channel_2 <= esc_loop_timer) &&
         (timer_channel_3 <= esc_loop_timer) &&
         (timer_channel_4 <= esc_loop_timer))
    {
      lowExit = 1;
    }

  }

  /* if(loop_counter == 0)Serial.print("Pitch: ");
        if(loop_counter == 1)Serial.print(angle_pitch, 0 );
        if(loop_counter == 2)Serial.print(" Roll: ");
        if(loop_counter == 3)Serial.print(angle_roll ,0);
        if(loop_counter == 4)Serial.print(" Yaw: ");
        if(loop_counter == 5)Serial.println(gyro_yaw / 65.5, 0 );

        loop_counter ++;
        if(loop_counter == 60)loop_counter = 0;
  */
/* Serial.print(esc_1); Serial.print("---");
  Serial.print(esc_2); Serial.print("---");
  Serial.print(esc_3); Serial.print("---"); Serial.println(esc_4);*/

}
void gps_setup(void) {
  ss.begin(4800);
  delay(500);
  // uint8_t Set_to_50Hz[14] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x01, 0x00, 0x2A, 0x32};//50hz
       //A small delay is added to give the GPS some time to respond @ 9600bps.
  //Set the refresh rate to 5Hz by using the ublox protocol.
  uint8_t Set_to_5Hz[14] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A};
  ss.write(Set_to_5Hz, 14);

   // uint8_t Set_to_10Hz[14] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12}; 
  //ss.write(Set_to_10Hz, 14);
  delay(350);

  //Disable GPGSV messages by using the ublox protocol.
  uint8_t Disable_GPGSV[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00,0x00,0x00,0x00,0x00, 0x01, 0x03,0x39}; 
  ss.write(Disable_GPGSV, 16);
  delay(350);
  uint8_t Disable_GPRMC[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00,0x00,0x00,0x00,0x00, 0x01, 0x04,0x40};
  ss.write(Disable_GPRMC, 16);
  delay(350);
  uint8_t Disable_GPVTG[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00,0x00,0x00,0x00,0x00, 0x01, 0x05,0x47};
  ss.write(Disable_GPVTG, 16);
  delay(350);

}

void EEPROMwriteAddr(int Waddress, byte val)
{
  EEPROM.beginTransmission(EEPROM_I2C_ADDRESS);
  EEPROM.write((int)(Waddress >> 8));   // MSB
  EEPROM.write((int)(Waddress & 0xFF)); // LSB


  EEPROM.write(val);
  EEPROM.endTransmission();

  delay(5);
}

byte EEPROMreadAddr(int Raddress)
{
  byte rData = 0xFF;

  EEPROM.beginTransmission(EEPROM_I2C_ADDRESS);

  EEPROM.write((int)(Raddress >> 8));   // MSB
  EEPROM.write((int)(Raddress & 0xFF)); // LSB
  EEPROM.endTransmission();


  EEPROM.requestFrom(EEPROM_I2C_ADDRESS, 1);

  rData =  EEPROM.read();

  return rData;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//This routine is called every time input 8, 9, 10 or 11 changed state. This is used to read the receiver signals.
//More information about this subroutine can be found in this video:
//https://youtu.be/bENjl1KQbvo
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ISR() {

  measured_time = micros() - measured_time_start;
  if (measured_time < 0)measured_time += 0xFFFF;
  measured_time_start = micros();
  if (measured_time > 3000)channel_select_counter = 0;
  else channel_select_counter++;

  if (channel_select_counter == 1)receiver_input[1] = measured_time;
  if (channel_select_counter == 2)receiver_input[2] = measured_time;
  if (channel_select_counter == 3)receiver_input[3] = measured_time;
  if (channel_select_counter == 4)receiver_input[4] = measured_time;
  if (channel_select_counter == 5)receiver_input[5] = measured_time;
  if (channel_select_counter == 6)receiver_input[6] = measured_time;

  // Clear out the interrupt pending bit, or the interrupt will
  // just happen again as soon as we return,
  // and we'll never get back into our loop() function.
  // These are write-one-to-clear.
  //GPIO_REG(GPIO_FALL_IP) = ppm_pinmask;//use if using FALLING
  GPIO_REG(GPIO_RISE_IP) = ppm_pinmask;
    
}
void receiverInput(void) {
  //get receiver's input
  receiver_input_channel_1 = convert_receiver_channel(1);                 //Convert the actual receiver signals for pitch to the standard 1000 - 2000us.
  receiver_input_channel_2 = convert_receiver_channel(2);                 //Convert the actual receiver signals for roll to the standard 1000 - 2000us.
  receiver_input_channel_3 = convert_receiver_channel(3);                 //Convert the actual receiver signals for throttle to the standard 1000 - 2000us.
  receiver_input_channel_4 = convert_receiver_channel(4);                 //Convert the actual receiver signals for yaw to the standard 1000 - 2000us.
  receiver_input_channel_5 = receiver_input[5];
  receiver_input_channel_6 = receiver_input[6];
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subroutine for reading the gyro
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void gyro_signalen() {
  //Read the MPU-6050
  if (eeprom_data[31] == 1) {
    Wire.beginTransmission(gyro_address);                                   //Start communication with the gyro.
    Wire.write(0x3B);                                                       //Start reading @ register 43h and auto increment with every read.
    Wire.endTransmission();                                                 //End the transmission.
    Wire.requestFrom(gyro_address, 14);                                     //Request 14 bytes from the gyro.

    receiverInput();

    while (Wire.available() < 14);                                          //Wait until the 14 bytes are received.
    acc_axis[1]  = Wire.read() << 8 | Wire.read();                           //Add the low and high byte to the acc_x variable.
    acc_axis[2]  = Wire.read() << 8 | Wire.read();                           //Add the low and high byte to the acc_y variable.
    acc_axis[3]  = Wire.read() << 8 | Wire.read();                           //Add the low and high byte to the acc_z variable.
    temperature  = Wire.read() << 8 | Wire.read();                           //Add the low and high byte to the temperature variable.
    gyro_axis[1] = Wire.read() << 8 | Wire.read();                          //Read high and low part of the angular data.
    gyro_axis[2] = Wire.read() << 8 | Wire.read();                          //Read high and low part of the angular data.
    gyro_axis[3] = Wire.read() << 8 | Wire.read();                          //Read high and low part of the angular data.
  }

  if (cal_int == 2000) {
    gyro_axis[1] -= gyro_axis_cal[1];                                       //Only compensate after the calibration.
    gyro_axis[2] -= gyro_axis_cal[2];                                       //Only compensate after the calibration.
    gyro_axis[3] -= gyro_axis_cal[3];                                       //Only compensate after the calibration.
  }
  gyro_roll = gyro_axis[eeprom_data[28] & 0b00000011];                      //Set gyro_roll to the correct axis that was stored in the EEPROM.
  if (eeprom_data[28] & 0b10000000)gyro_roll *= -1;                         //Invert gyro_roll if the MSB of EEPROM bit 28 is set.
  gyro_pitch = gyro_axis[eeprom_data[29] & 0b00000011];                     //Set gyro_pitch to the correct axis that was stored in the EEPROM.
  if (eeprom_data[29] & 0b10000000)gyro_pitch *= -1;                        //Invert gyro_pitch if the MSB of EEPROM bit 29 is set.
  gyro_yaw = gyro_axis[eeprom_data[30] & 0b00000011];                       //Set gyro_yaw to the correct axis that was stored in the EEPROM.
  if (eeprom_data[30] & 0b10000000)gyro_yaw *= -1;                          //Invert gyro_yaw if the MSB of EEPROM bit 30 is set.

  acc_x = acc_axis[eeprom_data[29] & 0b00000011];                           //Set acc_x to the correct axis that was stored in the EEPROM.
  if (eeprom_data[29] & 0b10000000)acc_x *= -1;                             //Invert acc_x if the MSB of EEPROM bit 29 is set.
  acc_y = acc_axis[eeprom_data[28] & 0b00000011];                           //Set acc_y to the correct axis that was stored in the EEPROM.
  if (eeprom_data[28] & 0b10000000)acc_y *= -1;                             //Invert acc_y if the MSB of EEPROM bit 28 is set.
  acc_z = acc_axis[eeprom_data[30] & 0b00000011];                           //Set acc_z to the correct axis that was stored in the EEPROM.
  if (eeprom_data[30] & 0b10000000)acc_z *= -1;                             //Invert acc_z if the MSB of EEPROM bit 30 is set.
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subroutine for calculating pid outputs
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//The PID controllers are explained in part 5 of the YMFC-3D video session:
//https://youtu.be/JBvnB0279-Q
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void calculate_pid() {
  //Roll calculations
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if (pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if (pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if (pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if (pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp;

  //Pitch calculations
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if (pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if (pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if (pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if (pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp;

  //Yaw calculations
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if (pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if (pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if (pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if (pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

  pid_last_yaw_d_error = pid_error_temp;
}

//This part converts the actual receiver signals to a standardized 1000 – 1500 – 2000 microsecond value.
//The stored data in the EEPROM is used.
int convert_receiver_channel(byte function) {
  byte channel, reverse;                                                       //First we declare some local variables
  int low, center, high, actual;
  int difference;

  channel = eeprom_data[function + 23] & 0b00000111;                           //What channel corresponds with the specific function
  if (eeprom_data[function + 23] & 0b10000000)reverse = 1;                     //Reverse channel when most significant bit is set
  else reverse = 0;                                                            //If the most significant is not set there is no reverse

  actual = receiver_input[channel];                                            //Read the actual receiver value for the corresponding function
  low = (eeprom_data[channel * 2 + 15] << 8) | eeprom_data[channel * 2 + 14];  //Store the low value for the specific receiver input channel
  center = (eeprom_data[channel * 2 - 1] << 8) | eeprom_data[channel * 2 - 2]; //Store the center value for the specific receiver input channel
  high = (eeprom_data[channel * 2 + 7] << 8) | eeprom_data[channel * 2 + 6];   //Store the high value for the specific receiver input channel

  if (actual < center) {                                                       //The actual receiver value is lower than the center value
    if (actual < low)actual = low;                                             //Limit the lowest value to the value that was detected during setup
    difference = ((long)(center - actual) * (long)500) / (center - low);       //Calculate and scale the actual value to a 1000 - 2000us value
    if (reverse == 1)return 1500 + difference;                                 //If the channel is reversed
    else return 1500 - difference;                                             //If the channel is not reversed
  }
  else if (actual > center) {                                                                      //The actual receiver value is higher than the center value
    if (actual > high)actual = high;                                           //Limit the lowest value to the value that was detected during setup
    difference = ((long)(actual - center) * (long)500) / (high - center);      //Calculate and scale the actual value to a 1000 - 2000us value
    if (reverse == 1)return 1500 - difference;                                 //If the channel is reversed
    else return 1500 + difference;                                             //If the channel is not reversed
  }
  else return 1500;
}

void set_gyro_registers() {
  //Setup the MPU-6050
  if (eeprom_data[31] == 1) {
    Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search.
    Wire.write(0x6B);                                                          //We want to write to the PWR_MGMT_1 register (6B hex)
    Wire.write(0x00);                                                          //Set the register bits as 00000000 to activate the gyro
    Wire.endTransmission();                                                    //End the transmission with the gyro.

    Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search.
    Wire.write(0x1B);                                                          //We want to write to the GYRO_CONFIG register (1B hex)
    Wire.write(0x08);                                                          //Set the register bits as 00001000 (500dps full scale)
    Wire.endTransmission();                                                    //End the transmission with the gyro

    Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search.
    Wire.write(0x1C);                                                          //We want to write to the ACCEL_CONFIG register (1A hex)
    Wire.write(0x10);                                                          //Set the register bits as 00010000 (+/- 8g full scale range)
    Wire.endTransmission();                                                    //End the transmission with the gyro

    //Let's perform a random register check to see if the values are written correct
    Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search
    Wire.write(0x1B);                                                          //Start reading @ register 0x1B
    Wire.endTransmission();                                                    //End the transmission
    Wire.requestFrom(gyro_address, 1);                                         //Request 1 bytes from the gyro
    while (Wire.available() < 1);                                              //Wait until the 6 bytes are received
    if (Wire.read() != 0x08) {                                                 //Check if the value is 0x08
      digitalWrite(STATUS_LED , LOW);                                                  //Turn on the warning led
      while (1)delay(10);                                                      //Stay in this loop for ever
    }

    Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search
    Wire.write(0x1A);                                                          //We want to write to the CONFIG register (1A hex)
    Wire.write(0x03);                                                          //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
    Wire.endTransmission();                                                    //End the transmission with the gyro

  }
}
void read_gps(void) {

  while (ss.available() && new_line_found == 0) {                                                   //Stay in this loop as long as there is serial information from the GPS available.
    char read_serial_byte = ss.read();                                                              //Load a new serial byte in the read_serial_byte variable.
     Serial.print(read_serial_byte);
    if (read_serial_byte == '$') {                                                                       //If the new byte equals a $ character.
      for (message_counter = 0; message_counter <= 99; message_counter ++) {                             //Clear the old data from the incomming buffer array.    
        incomming_message[message_counter] = '-';                                                        //Write a - at every position.
      }
      message_counter = 0;                                                                               //Reset the message_counter variable because we want to start writing at the begin of the array.
    }
    else if (message_counter <= 99)message_counter ++;                                                   //If the received byte does not equal a $ character, increase the message_counter variable.
    incomming_message[message_counter] = read_serial_byte;                                               //Write the new received byte to the new position in the incomming_message array.
   
    if (read_serial_byte == '*') new_line_found = 1;                                                     //Every NMEA line end with a *. If this character is detected the new_line_found variable is set to 1.
  }

  //If the software has detected a new NMEA line it will check if it's a valid line that can be used.
  if (new_line_found == 1) {                                                                             //If a new NMEA line is found.
    new_line_found = 0;
    digitalWrite(GPS_LED, HIGH); //Reset the new_line_found variable for the next line.
    if (incomming_message[4] == 'L' && incomming_message[5] == 'L' && incomming_message[7] == ',') {     //When there is no GPS fix or latitude/longitude information available.
      digitalWrite(GPS_LED, LOW);                                    //Change the LED on the STM32 to indicate GPS reception.
      //Set some variables to 0 if no valid information is found by the GPS module. This is needed for GPS lost when flying.
      l_lat_gps = 0;
      l_lon_gps = 0;
      lat_gps_previous = 0;
      lon_gps_previous = 0;
      number_used_sats = 0;

    }
    //If the line starts with GA and if there is a GPS fix we can scan the line for the latitude, longitude and number of satellites.
    if (incomming_message[4] == 'G' && incomming_message[5] == 'A' && (incomming_message[44] == '1' || incomming_message[44] == '2')) {
      lat_gps_actual = ((int)incomming_message[19] - 48) *  (long)10000000;                              //Filter the minutes for the GGA line multiplied by 10.
      lat_gps_actual += ((int)incomming_message[20] - 48) * (long)1000000;                               //Filter the minutes for the GGA line multiplied by 10.
      lat_gps_actual += ((int)incomming_message[22] - 48) * (long)100000;                                //Filter the minutes for the GGA line multiplied by 10.
      lat_gps_actual += ((int)incomming_message[23] - 48) * (long)10000;                                 //Filter the minutes for the GGA line multiplied by 10.
      lat_gps_actual += ((int)incomming_message[24] - 48) * (long)1000;                                  //Filter the minutes for the GGA line multiplied by 10.
      lat_gps_actual += ((int)incomming_message[25] - 48) * (long)100;                                   //Filter the minutes for the GGA line multiplied by 10.
      lat_gps_actual += ((int)incomming_message[26] - 48) * (long)10;                                    //Filter the minutes for the GGA line multiplied by 10.
      lat_gps_actual /= (long)6;                                                                         //To convert the minutes to degrees we need to divide the minutes by 6.
      lat_gps_actual += ((int)incomming_message[17] - 48) *  (long)100000000;                            //Add the degrees multiplied by 10.
      lat_gps_actual += ((int)incomming_message[18] - 48) *  (long)10000000;                             //Add the degrees multiplied by 10.
      lat_gps_actual /= 10;                                                                              //Divide everything by 10.

      lon_gps_actual = ((int)incomming_message[33] - 48) *  (long)10000000;                              //Filter the minutes for the GGA line multiplied by 10.
      lon_gps_actual += ((int)incomming_message[34] - 48) * (long)1000000;                               //Filter the minutes for the GGA line multiplied by 10.
      lon_gps_actual += ((int)incomming_message[36] - 48) * (long)100000;                                //Filter the minutes for the GGA line multiplied by 10.
      lon_gps_actual += ((int)incomming_message[37] - 48) * (long)10000;                                 //Filter the minutes for the GGA line multiplied by 10.
      lon_gps_actual += ((int)incomming_message[38] - 48) * (long)1000;                                  //Filter the minutes for the GGA line multiplied by 10.
      lon_gps_actual += ((int)incomming_message[39] - 48) * (long)100;                                   //Filter the minutes for the GGA line multiplied by 10.
      lon_gps_actual += ((int)incomming_message[40] - 48) * (long)10;                                    //Filter the minutes for the GGA line multiplied by 10.
      lon_gps_actual /= (long)6;                                                                         //To convert the minutes to degrees we need to divide the minutes by 6.
      lon_gps_actual += ((int)incomming_message[30] - 48) * (long)1000000000;                            //Add the degrees multiplied by 10.
      lon_gps_actual += ((int)incomming_message[31] - 48) * (long)100000000;                             //Add the degrees multiplied by 10.
      lon_gps_actual += ((int)incomming_message[32] - 48) * (long)10000000;                              //Add the degrees multiplied by 10.
      lon_gps_actual /= 10;                                                                              //Divide everything by 10.

      if (incomming_message[28] == 'N')latitude_north = 1;                                               //When flying north of the equator the latitude_north variable will be set to 1.
      else latitude_north = 0;                                                                           //When flying south of the equator the latitude_north variable will be set to 0.

      if (incomming_message[42] == 'E')longiude_east = 1;                                                //When flying east of the prime meridian the longiude_east variable will be set to 1.
      else longiude_east = 0;                                                                            //When flying west of the prime meridian the longiude_east variable will be set to 0.

      number_used_sats = ((int)incomming_message[46] - 48) * (long)10;                                   //Filter the number of satillites from the GGA line.
      number_used_sats += (int)incomming_message[47] - 48;                                               //Filter the number of satillites from the GGA line.


      if (lat_gps_previous == 0 && lon_gps_previous == 0) {                                              //If this is the first time the GPS code is used.
        lat_gps_previous = lat_gps_actual;                                                               //Set the lat_gps_previous variable to the lat_gps_actual variable.
        lon_gps_previous = lon_gps_actual;                                                               //Set the lon_gps_previous variable to the lon_gps_actual variable.
      }

      lat_gps_loop_add = (float)(lat_gps_actual - lat_gps_previous) / 10.0;                              //Divide the difference between the new and previous latitude by ten.
      lon_gps_loop_add = (float)(lon_gps_actual - lon_gps_previous) / 10.0;                              //Divide the difference between the new and previous longitude by ten.

      l_lat_gps = lat_gps_previous;                                                                      //Set the l_lat_gps variable to the previous latitude value.
      l_lon_gps = lon_gps_previous;                                                                      //Set the l_lon_gps variable to the previous longitude value.

      lat_gps_previous = lat_gps_actual;                                                                 //Remember the new latitude value in the lat_gps_previous variable for the next loop.
      lon_gps_previous = lon_gps_actual;                                                                 //Remember the new longitude value in the lat_gps_previous variable for the next loop.

      //The GPS is set to a 5Hz refresh rate. Between every 2 GPS measurments, 9 GPS values are simulated.
      gps_add_counter = 5;                                                                               //Set the gps_add_counter variable to 5 as a count down loop timer
      new_gps_data_counter = 9;                                                                          //Set the new_gps_data_counter to 9. This is the number of simulated values between 2 GPS measurements.
      lat_gps_add = 0;                                                                                   //Reset the lat_gps_add variable.
      lon_gps_add = 0;                                                                                   //Reset the lon_gps_add variable.
      new_gps_data_available = 1;                                                                        //Set the new_gps_data_available to indicate that there is new data available.
    }

    //If the line starts with SA and if there is a GPS fix we can scan the line for the fix type (none, 2D or 3D).
    if (incomming_message[4] == 'S' && incomming_message[5] == 'A') {
      fix_type = (int)incomming_message[9] - 48;
      if (((int)incomming_message[9] - 48) == 3) {
        digitalWrite(GPS_LED , LOW);
       
       Serial.print("LAT:");Serial.println(lat_gps_actual);
       Serial.print("LONG:");Serial.println(lon_gps_actual);
       /*Serial.print(receiver_input[1]); Serial.print("---");
       Serial.print(receiver_input[2]); Serial.print("---");
       Serial.print(receiver_input[3]); Serial.print("---"); Serial.println(receiver_input[4]);*/
      }
      else if (((int)incomming_message[9] - 48) == 1) {
        digitalWrite(GPS_LED , HIGH);
      }
    }

  }

  //After 5 program loops 5 x 4ms = 20ms the gps_add_counter is 0.
  if (gps_add_counter == 0 && new_gps_data_counter > 0) {                                                 //If gps_add_counter is 0 and there are new GPS simulations needed.
    new_gps_data_available = 1;                                                                           //Set the new_gps_data_available to indicate that there is new data available.
    new_gps_data_counter --;                                                                              //Decrement the new_gps_data_counter so there will only be 9 simulations
    gps_add_counter = 5;                                                                                  //Set the gps_add_counter variable to 5 as a count down loop timer

    lat_gps_add += lat_gps_loop_add;                                                                      //Add the simulated part to a buffer float variable because the l_lat_gps can only hold integers.
    if (abs(lat_gps_add) >= 1) {                                                                          //If the absolute value of lat_gps_add is larger then 1.
      l_lat_gps += (int)lat_gps_add;                                                                      //Increment the lat_gps_add value with the lat_gps_add value as an integer. So no decimal part.
      lat_gps_add -= (int)lat_gps_add;                                                                    //Subtract the lat_gps_add value as an integer so the decimal value remains.
    }

    lon_gps_add += lon_gps_loop_add;                                                                      //Add the simulated part to a buffer float variable because the l_lat_gps can only hold integers.
    if (abs(lon_gps_add) >= 1) {                                                                          //If the absolute value of lat_gps_add is larger then 1.
      l_lon_gps += (int)lon_gps_add;                                                                      //Increment the lat_gps_add value with the lat_gps_add value as an integer. So no decimal part.
      lon_gps_add -= (int)lon_gps_add;                                                                    //Subtract the lat_gps_add value as an integer so the decimal value remains.
    }
  }

  if (new_gps_data_available) {                                                                           //If there is a new set of GPS data available.
    digitalWrite(GPS_LED, HIGH);
    if (number_used_sats < 8)digitalWrite(GPS_LED, LOW);                //Change the LED on the STM32 to indicate GPS reception.
    else digitalWrite(GPS_LED, LOW);                                                              //Turn the LED on the STM solid on (LED function is inverted). Check the STM32 schematic.
   // gps_watchdog_timer = millis();                                                                        //Reset the GPS watch dog tmer.
    new_gps_data_available = 0;                                                                           //Reset the new_gps_data_available variable.

    if (flight_mode == 1 && waypoint_set == 0) {                                                          //If the flight mode is 3 (GPS hold) and no waypoints are set.
      waypoint_set = 1;                                                                                   //Indicate that the waypoints are set.
      l_lat_waypoint = l_lat_gps;                                                                         //Remember the current latitude as GPS hold waypoint.
      l_lon_waypoint = l_lon_gps;                                                                         //Remember the current longitude as GPS hold waypoint.
    }

    if (flight_mode == 1 && waypoint_set == 1) {                                                          //If the GPS hold mode and the waypoints are stored.
      gps_lon_error = l_lon_waypoint - l_lon_gps;                                                         //Calculate the latitude error between waypoint and actual position.
      gps_lat_error = l_lat_gps - l_lat_waypoint;                                                         //Calculate the longitude error between waypoint and actual position.

      gps_lat_total_avarage -=  gps_lat_rotating_mem[ gps_rotating_mem_location];                         //Subtract the current memory position to make room for the new value.
      gps_lat_rotating_mem[ gps_rotating_mem_location] = gps_lat_error - gps_lat_error_previous;          //Calculate the new change between the actual pressure and the previous measurement.
      gps_lat_total_avarage +=  gps_lat_rotating_mem[ gps_rotating_mem_location];                         //Add the new value to the long term avarage value.

      gps_lon_total_avarage -=  gps_lon_rotating_mem[ gps_rotating_mem_location];                         //Subtract the current memory position to make room for the new value.
      gps_lon_rotating_mem[ gps_rotating_mem_location] = gps_lon_error - gps_lon_error_previous;          //Calculate the new change between the actual pressure and the previous measurement.
      gps_lon_total_avarage +=  gps_lon_rotating_mem[ gps_rotating_mem_location];                         //Add the new value to the long term avarage value.
      gps_rotating_mem_location++;                                                                        //Increase the rotating memory location.
      if ( gps_rotating_mem_location == 35) gps_rotating_mem_location = 0;                                //Start at 0 when the memory location 35 is reached.

      gps_lat_error_previous = gps_lat_error;                                                             //Remember the error for the next loop.
      gps_lon_error_previous = gps_lon_error;                                                             //Remember the error for the next loop.

      //Calculate the GPS pitch and roll correction as if the nose of the multicopter is facing north.
      //The Proportional part = (float)gps_lat_error * gps_p_gain.
      //The Derivative part = (float)gps_lat_total_avarage * gps_d_gain.
      gps_pitch_adjust_north = (float)gps_lat_error * gps_p_gain + (float)gps_lat_total_avarage * gps_d_gain;
      gps_roll_adjust_north = (float)gps_lon_error * gps_p_gain + (float)gps_lon_total_avarage * gps_d_gain;

      if (!latitude_north)gps_pitch_adjust_north *= -1;                                                   //Invert the pitch adjustmet because the quadcopter is flying south of the equator.
      if (!longiude_east)gps_roll_adjust_north *= -1;                                                     //Invert the roll adjustmet because the quadcopter is flying west of the prime meridian.

      //Because the correction is calculated as if the nose was facing north, we need to convert it for the current heading.
      gps_roll_adjust = ((float)gps_roll_adjust_north * cos(angle_yaw * 0.017453)) + ((float)gps_pitch_adjust_north * cos((angle_yaw - 90) * 0.017453));
      gps_pitch_adjust = ((float)gps_pitch_adjust_north * cos(angle_yaw * 0.017453)) + ((float)gps_roll_adjust_north * cos((angle_yaw + 90) * 0.017453));

      //Limit the maximum correction to 300. This way we still have full controll with the pitch and roll stick on the transmitter.
      if (gps_roll_adjust > 300) gps_roll_adjust = 300;
      if (gps_roll_adjust < -300) gps_roll_adjust = -300;
      if (gps_pitch_adjust > 300) gps_pitch_adjust = 300;
      if (gps_pitch_adjust < -300) gps_pitch_adjust = -300;
    }
  }

  /*  if (gps_watchdog_timer + 1000 < millis()) {                                                             //If the watchdog timer is exceeded the GPS signal is missing.
      if (flight_mode >= 3 && start > 0) {                                                                  //If flight mode is set to 3 (GPS hold).
        flight_mode = 2;                                                                                    //Set the flight mode to 2.
        error = 4;                                                                                          //Output an error.
      }
    }
  */
  if (flight_mode == 0 && waypoint_set > 0) {                                                              //If the GPS hold mode is disabled and the waypoints are set.
    gps_roll_adjust = 0;                                                                                  //Reset the gps_roll_adjust variable to disable the correction.
    gps_pitch_adjust = 0;                                                                                 //Reset the gps_pitch_adjust variable to disable the correction.
    if (waypoint_set == 1) {                                                                              //If the waypoints are stored
      gps_rotating_mem_location = 0;                                                                      //Set the gps_rotating_mem_location to zero so we can empty the
      waypoint_set = 2;                                                                                   //Set the waypoint_set variable to 2 as an indication that the buffer is not cleared.
    }
    gps_lon_rotating_mem[ gps_rotating_mem_location] = 0;                                                 //Reset the current gps_lon_rotating_mem location.
    gps_lat_rotating_mem[ gps_rotating_mem_location] = 0;                                                 //Reset the current gps_lon_rotating_mem location.
    gps_rotating_mem_location++;                                                                          //Increment the gps_rotating_mem_location variable for the next loop.
    if (gps_rotating_mem_location == 36) {                                                                //If the gps_rotating_mem_location equals 36, all the buffer locations are cleared.
      waypoint_set = 0;                                                                                   //Reset the waypoint_set variable to 0.
      //Reset the variables that are used for the D-controller.
      gps_lat_error_previous = 0;
      gps_lon_error_previous = 0;
      gps_lat_total_avarage = 0;
      gps_lon_total_avarage = 0;
      gps_rotating_mem_location = 0;
    }
  }
}
