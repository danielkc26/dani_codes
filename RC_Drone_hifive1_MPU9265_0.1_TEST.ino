//UPDATED 13/11/2019

#include "encoding.h"
#include <SlowSoftI2CMaster.h>
#include <SlowSoftWire.h>

#include <SPI.h>
#include <MPU9250.h>

#define SPI_CLOCK 8000000  // 8MHz clock works.
#define INT_PIN  9 
#define SS_PIN   10 

MPU9250 mpu(SPI_CLOCK, SS_PIN);//9265 is 9250 compatible

SlowSoftWire EEPROM = SlowSoftWire(18,19);

#define EEPROM_I2C_ADDRESS 0x50
#define ADC_CHANNEL        1
#define LED                3
#define LED2               2

#include <Adafruit_MCP3008.h>

Adafruit_MCP3008 adc;

int ppmPin = 17;
int ppmPinMsk;


uint8_t response_acc[6];
uint8_t response_gyro[6];
uint8_t response_temp[2];

int16_t bit_data;
float MPUdata;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_roll =0.70;                 //Gain setting for the roll P-controller
float pid_i_gain_roll =0.0010;               //Gain setting for the roll I-controller
float pid_d_gain_roll = 3.75;                //Gain setting for the roll D-controller

int pid_max_roll = 400;                      //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;    //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;    //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;    //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;            //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 11.0 ;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.05;                 //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 1.75;                 //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;                       //Maximum output of the PID-controller (+/-)

boolean auto_level = true;                   //Auto level on (true) or off (false)

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
byte eeprom_data[36];
byte highByte, lowByte;
volatile uint8_t channel_select_counter;
volatile int32_t measured_time, measured_time_start;
volatile int32_t receiver_input_channel_1, 
             receiver_input_channel_2,
             receiver_input_channel_3,
             receiver_input_channel_4,
             receiver_input_channel_5,
             receiver_input_channel_6;

volatile int32_t receiver_input[8];
             
int esc_1, esc_2, esc_3, esc_4;
int throttle, battery_voltage;
int cal_int, start, gyro_address;

int16_t temperature;
int16_t acc_axis[4], gyro_axis[4];
float roll_level_adjust, pitch_level_adjust;

int16_t acc_x, acc_y, acc_z;
int acc_total_vector;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer,esc_loop_timer1,esc_loop_timer2,esc_loop_timer3;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
unsigned long loop_timer,loop_timer1,loop_timer2,loop_timer3,loop_timer4,loop_counter;
int gyro_pitch, gyro_roll, gyro_yaw;
int gyro_axis_cal[4];
volatile float pid_error_temp;
volatile float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
volatile float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
volatile float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
boolean gyro_angles_set;

uint32_t pin4 = 4;uint32_t pin5 = 5;uint32_t pin6 = 6;uint32_t pin7 = 7;
uint32_t bitmask4 = digitalPinToBitMask(pin4);
uint32_t bitmask5 = digitalPinToBitMask(pin5);
uint32_t bitmask6 = digitalPinToBitMask(pin6);
uint32_t bitmask7 = digitalPinToBitMask(pin7);

int channel=ADC_CHANNEL;
int lowExit=0;
int j=0;

//PPM method 2
unsigned long t[8];
int pulse=0;

//PPM method 3
const int SIGNAL_COUNT = 7;

volatile unsigned long widths [SIGNAL_COUNT + 1];
volatile byte count;

unsigned long lastPulse;

volatile int channel_Num=1;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup routine
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup(){
            
  Serial.begin(57600);
  pinMode(LED,OUTPUT);
  pinMode(LED2,OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(INT_PIN, INPUT);
  pinMode(ppmPin, INPUT);

  EEPROM.begin();
  delay(250);
  
 

 // Hardware SPI
  adc.begin(8);//uses (13, 11, 12, 8-cs);
  delay(250);
  SPI.begin();
  delay(250);
  
 
  ppmPinMsk = digitalPinToBitMask(ppmPin);
  
  delay(2000);
  Serial.println(">> Firmware Version 0.1<<");
  Serial.println(">> START");
  //Copy the EEPROM data for fast access data.
  for(start = 0; start <= 35; start++)eeprom_data[start] = EEPROMreadAddr(start);
  start = 0;                                                                //Set start back to zero.
  gyro_address = eeprom_data[32];                                           //Store the gyro address in the variable.

  //Use the led on the Arduino for startup indication.
  digitalWrite(LED,LOW);                                                    //Turn on the warning led.

  //Check the EEPROM signature to make sure that the setup program is executed.
  while(eeprom_data[33] != 'J' || eeprom_data[34] != 'M' || eeprom_data[35] != 'B')delay(10);
  Serial.println(">> START-EEPROM");
  //The flight controller needs the MPU-6050 with gyro and accelerometer
  //If setup is completed without MPU-6050 stop the flight controller program  
  if(eeprom_data[31] == 2 || eeprom_data[31] == 3)delay(10);
  
  Serial.println(">> START-GYRO");
 
  set_gyro_registers();                                                     //Set the specific gyro registers.
  
  Serial.println(">> GYRO-STARTED");
  Serial.print(">> GPIO-SETTINGS");
 
  for (cal_int = 0; cal_int < 1250 ; cal_int ++){                           //Wait 5 seconds before continuing.
    if(cal_int % 125 == 0){
     // digitalWrite(LED, !digitalRead(LED));   //Change the led status to indicate calibration.
      Serial.print(".");
    }
  //Set digital port 4, 5, 6 and 7 high.                                                    
    GPIO_REG(GPIO_OUTPUT_VAL) |= bitmask4 | bitmask5 | bitmask6| bitmask7 ;   //HIGH  
    delayMicroseconds(1000);                                                  //Wait 1000us.                                           
     //Set digital port 4, 5, 6 and 7 low.
    GPIO_REG(GPIO_OUTPUT_VAL) &= ~bitmask4 & (~bitmask5) & (~bitmask6) & (~bitmask7);//LOW
    delayMicroseconds(3000);                                                  //Wait 3000us.
  }
  
 Serial.println("."); 
 Serial.println(">> GYRO CALIBRATION");
  
  //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
 Serial.print(">> CALIBRATING");
  
  for (cal_int = 0; cal_int < 2000 ; cal_int ++){                         //Take 2000 readings for calibration.
  
    if(cal_int % 125 == 0){
           // digitalWrite(LED, !digitalRead(LED));   //Change the led status to indicate calibration.
            Serial.print(".");
     }
    digitalWrite(LED, LOW);      

    if(cal_int % 15 == 0)digitalWrite(LED, HIGH);                //Change the led status to indicate calibration.

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
  Serial.println("."); 
  Serial.println(">> CALIBRATED");
  //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
  gyro_axis_cal[1] /= 2000;                                               //Divide the roll total by 2000.
  gyro_axis_cal[2] /= 2000;                                               //Divide the pitch total by 2000.
  gyro_axis_cal[3] /= 2000;                                               //Divide the yaw total by 2000.

 
  Serial.println(">> RECEIVER START");
  attachInterrupt(digitalPinToInterrupt(ppmPin), ISR, RISING);
  set_csr(mstatus, MSTATUS_MIE); 

  //Wait until the receiver is active and the throtle is set to the lower position.
  while(receiver_input_channel_3 < 990 || receiver_input_channel_3 > 1020 || receiver_input_channel_4 < 1400){
  //Serial.print(receiver_input[3]);Serial.print("-");Serial.println(receiver_input_channel_3);
  receiver_input_channel_3 = convert_receiver_channel(3);                 //Convert the actual receiver signals for throttle to the standard 1000 - 2000us
  receiver_input_channel_4 = convert_receiver_channel(4);                 //Convert the actual receiver signals for yaw to the standard 1000 - 2000us
  start ++;                                                               //While waiting increment start whith every loop.
  
  //We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while waiting for the receiver inputs.
  
  //Set digital poort 4, 5, 6 and 7 high.
  GPIO_REG(GPIO_OUTPUT_VAL) |= bitmask4 | bitmask5 | bitmask6| bitmask7 ;//HIGH
  delayMicroseconds(1000);                                                //Wait 1000us.
  
  //Set digital poort 4, 5, 6 and 7 low.
  GPIO_REG(GPIO_OUTPUT_VAL) &= ~bitmask4 & (~bitmask5) & (~bitmask6) & (~bitmask7);//LOW
  delay(3);                                                               //Wait 3 milliseconds before the next loop.
    digitalWrite(LED, LOW);
    if(start == 125){                                                       //Every 125 loops (500ms).
      digitalWrite(LED, HIGH);                                   //Change the led status.
      start = 0;                                                            //Start again at 0.
    }
  }
  start = 0;                                                                //Set start back to 0.

//HIFIVE1 does not have analog inputs , hence we use an ADC. Here MCP3008 is used  
  //Load the battery voltage to the battery_voltage variable.
  //65 is the voltage compensation for the diode. 
  //12.6V equals ~5V @ Analog 0.
  //12.6V equals 1023 analogRead(0).
  //1260 / 1023 = 1.2317.
  //The variable battery_voltage holds 1050 if the battery voltage is 10.5V.
//  battery_voltage = (analogRead(0) + 65) * 1.2317;
 // Serial.print("adc-");Serial.println(adc.readADC(channel));
  
  battery_voltage = (adc.readADC(channel)+65) * 1.2317; // no diode is used
  
  loop_timer = micros();                                                    //Set the timer for the next loop.
  //When everything is done, turn off the led.
  digitalWrite(LED,HIGH); //Turn off the warning led.
  Serial.println(">> SETUP DONE");

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int timer=0;
int h=0;
void loop(){

  receiver_input_channel_1 = convert_receiver_channel(1);                 //Convert the actual receiver signals for pitch to the standard 1000 - 2000us.
  receiver_input_channel_2 = convert_receiver_channel(2);                 //Convert the actual receiver signals for roll to the standard 1000 - 2000us.
  receiver_input_channel_3 = convert_receiver_channel(3);                 //Convert the actual receiver signals for throttle to the standard 1000 - 2000us.
  receiver_input_channel_4 = convert_receiver_channel(4);                 //Convert the actual receiver signals for yaw to the standard 1000 - 2000us.

  //65.5 = 1 deg/sec (check the datasheet of the MPU-6050 for more information).
  gyro_roll_input = (gyro_roll_input * 0.7) + (((float)gyro_roll / 65.5) * 0.3);   //Gyro pid input is deg/sec.
  gyro_pitch_input = (gyro_pitch_input * 0.7) + (((float)gyro_pitch / 65.5) * 0.3);//Gyro pid input is deg/sec.
  gyro_yaw_input = (gyro_yaw_input * 0.7) + (((float)gyro_yaw / 65.5) * 0.3);      //Gyro pid input is deg/sec.

  //Gyro angle calculations
  //for 250HZ
  //0.0000611 = 1 / 250Hz / 65.5
  angle_pitch += (float)gyro_pitch * 0.0000611;                                    //Calculate the traveled pitch angle and add this to the angle_pitch variable.
  angle_roll += (float)gyro_roll * 0.0000611;                                      //Calculate the traveled roll angle and add this to the angle_roll variable. 

 //for 250HZ
  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch -= angle_roll * sin((float)gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the roll angle to the pitch angel.
  angle_roll += angle_pitch * sin((float)gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the pitch angle to the roll angel.
 

  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));       //Calculate the total accelerometer vector.
  
  if(abs(acc_y) < acc_total_vector){                                        //Prevent the asin function to produce a NaN
    angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;          //Calculate the pitch angle.
  }
  if(abs(acc_x) < acc_total_vector){                                        //Prevent the asin function to produce a NaN
    angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;          //Calculate the roll angle.
  }
 
  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration.
  angle_pitch_acc -= 0.0;                                                   //Accelerometer calibration value for pitch.
  angle_roll_acc -= 0.00;                                                    //Accelerometer calibration value for roll.
  
  
  angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;            //Correct the drift of the gyro pitch angle with the accelerometer pitch angle. 
  angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;               //Correct the drift of the gyro roll angle with the accelerometer roll angle.

  
  pitch_level_adjust = angle_pitch * 15;                                    //Calculate the pitch angle correction
  roll_level_adjust = angle_roll * 15;                                      //Calculate the roll angle correction

  if(!auto_level){                                                          //If the quadcopter is not in auto-level mode
    pitch_level_adjust = 0;                                                 //Set the pitch angle correction to zero.
    roll_level_adjust = 0;                                                  //Set the roll angle correcion to zero.
  }

  //For starting the motors: throttle low and yaw left (step 1).

  if(receiver_input_channel_3 < 1050 && receiver_input_channel_4 < 1050)start = 1;

  //When yaw stick is back in the center position start the motors (step 2).
  if(start == 1 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1450){
    start = 2;
    
    angle_pitch = angle_pitch_acc;                                          //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
    angle_roll = angle_roll_acc;                                            //Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started.
    //gyro_angles_set = true;                                                 //Set the IMU started flag.

    //Reset the PID controllers for a bumpless start.
    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_i_mem_yaw = 0;
    pid_last_yaw_d_error = 0;
  }
  //Stopping the motors: throttle low and yaw right.
  if(start == 2 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1950)start = 0;

  //The PID set point in degrees per second is determined by the roll receiver input.
  //In the case of deviding by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_roll_setpoint = 0;
  //We need a little dead band of 16us for better results. 
  if(receiver_input_channel_1 > 1508)pid_roll_setpoint = receiver_input_channel_1 - 1508;
  else if(receiver_input_channel_1 < 1492)pid_roll_setpoint = receiver_input_channel_1 - 1492;

  pid_roll_setpoint -= roll_level_adjust;                                   //Subtract the angle correction from the standardized receiver roll input value.
  pid_roll_setpoint /= 3.0;                                                 //Divide the setpoint for the PID roll controller by 3 to get angles in degrees.


  //The PID set point in degrees per second is determined by the pitch receiver input.
  //In the case of deviding by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_pitch_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if(receiver_input_channel_2 > 1508)pid_pitch_setpoint = receiver_input_channel_2 - 1508;
  else if(receiver_input_channel_2 < 1492)pid_pitch_setpoint = receiver_input_channel_2 - 1492;

  pid_pitch_setpoint -= pitch_level_adjust;                                  //Subtract the angle correction from the standardized receiver pitch input value.
  pid_pitch_setpoint /= 3.0;                                                 //Divide the setpoint for the PID pitch controller by 3 to get angles in degrees.

  //The PID set point in degrees per second is determined by the yaw receiver input.
  //In the case of deviding by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_yaw_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if(receiver_input_channel_3 > 1050){ //Do not yaw when turning off the motors.
    if(receiver_input_channel_4 > 1508)pid_yaw_setpoint = (receiver_input_channel_4 - 1508)/3.0;
    else if(receiver_input_channel_4 < 1492)pid_yaw_setpoint = (receiver_input_channel_4 - 1492)/3.0;
  }
  
  calculate_pid();                                                      //PID inputs are known. So we can calculate the pid output.
 
  //The battery voltage is needed for compensation.
  //A complementary filter is used to reduce noise.
  //0.09853 = 0.08 * 1.2317.
  battery_voltage = battery_voltage * 0.92 + (adc.readADC(channel)+65) * 0.09853;
  //Turn on the led if battery voltage is to low.
  //Serial.println(battery_voltage);
///  if(battery_voltage < 1000 && battery_voltage > 600)digitalWrite(LED, LOW);
  
  throttle = receiver_input_channel_3;                                      //We need the throttle signal as a base signal.
  
  if (start == 2){  
  
    if (throttle > 1800) throttle = 1800;                        //previously 1800           //We need some room to keep full control at full throttle.
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
   
    if (esc_1 < 1100) esc_1 = 1100;                                         //Keep the motors running.
    if (esc_2 < 1100) esc_2 = 1100;                                         //Keep the motors running.
    if (esc_3 < 1100) esc_3 = 1100;                                         //Keep the motors running.
    if (esc_4 < 1100) esc_4 = 1100;                                         //Keep the motors running.
    
    if(esc_1 > 2000)esc_1 = 2000;                                           //Limit the esc-1 pulse to 2000us.
    if(esc_2 > 2000)esc_2 = 2000;                                           //Limit the esc-2 pulse to 2000us.
    if(esc_3 > 2000)esc_3 = 2000;                                           //Limit the esc-3 pulse to 2000us.
    if(esc_4 > 2000)esc_4 = 2000;                                           //Limit the esc-4 pulse to 2000us.  
  }
  else{
  
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


   //if(micros() - loop_timer > 4050)digitalWrite(LED, LOW);                   //Turn on the LED if the loop time exceeds 4050us.
 // if(h==500)Serial.println(micros()-loop_timer);
 
 lowExit=0;
 loop_timer1 = micros();
  
  GPIO_REG(GPIO_OUTPUT_VAL) |= bitmask4 | bitmask5 | bitmask6| bitmask7 ;//HIGH   //Set digital outputs 4,5,6 and 7 high.                                                      
  timer_channel_1 = esc_1 + loop_timer1;                                     //Calculate the time of the faling edge of the esc-1 pulse.
  timer_channel_2 = esc_2 + loop_timer1;                                     //Calculate the time of the faling edge of the esc-2 pulse.
  timer_channel_3 = esc_3 + loop_timer1;                                     //Calculate the time of the faling edge of the esc-3 pulse.
  timer_channel_4 = esc_4 + loop_timer1;                                     //Calculate the time of the faling edge of the esc-4 pulse.
  
 
  //There is always 1000us of spare time. So let's do something usefull that is very time consuming.
  //Get the current gyro and receiver data and scale it to degrees per second for the pid calculations.
    gyro_signalen();
     
   while(lowExit<1){//Stay in this loop until output 4,5,6 and 7 are low. 
     esc_loop_timer = micros();                                         //Read the current time.
    if(timer_channel_1 <= esc_loop_timer){ GPIO_REG(GPIO_OUTPUT_VAL) &= ~bitmask4 ;}//digitalWrite(4,LOW);}                //Set digital output 4 to low if the time is expired.
     
    if(timer_channel_2 <= esc_loop_timer ){ GPIO_REG(GPIO_OUTPUT_VAL) &= ~bitmask5;}//digitalWrite(5,LOW);}                //Set digital output 5 to low if the time is expired.
     
    if(timer_channel_3 <= esc_loop_timer  ){ GPIO_REG(GPIO_OUTPUT_VAL) &= ~bitmask6;}// digitalWrite(6,LOW);}                //Set digital output 6 to low if the time is expired.
    
    if(timer_channel_4 <= esc_loop_timer ){ GPIO_REG(GPIO_OUTPUT_VAL) &= ~bitmask7;}//digitalWrite(7,LOW);lowExit=1;}                //Set digital output 7 to low if the time is expired.
    
     if( (timer_channel_1 <= esc_loop_timer)&& 
         (timer_channel_2 <= esc_loop_timer)&&
         (timer_channel_3 <= esc_loop_timer)&& 
         (timer_channel_4 <= esc_loop_timer))
     {
      lowExit=1;
     }
    
  }
    
 while (micros() - loop_timer < 4000);
 loop_timer = micros();

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

void ISR(){

  //METHOD 3
//noInterrupts();
//clear_csr(mstatus, MSTATUS_MIE);

  unsigned long now = micros();
 
  // a long gap means we start again
  if ((now - lastPulse) >= 2000)
    count = 0;
  lastPulse = now;
  if (count >= (SIGNAL_COUNT +1))
    return;
    
  
  widths [count++] = now;

   if(channel_Num < count ){
   
    //  Serial.print (" Ch ");
    //  Serial.print (channel_Num);
       // Serial.print (" - ");
    // Serial.print (widths [channel_Num] - widths [channel_Num - 1]);
       receiver_input[channel_Num]=widths [channel_Num] - widths [channel_Num - 1];
      
    //  Serial.print(" - ");Serial.print(receiver_input[channel_Num]);
       channel_Num++;
  }
 if (count >= SIGNAL_COUNT)
    {
    count = 0;
    channel_Num=1;
   
    Serial.println();
        
    }
  

GPIO_REG(GPIO_RISE_IP) = ppmPinMsk;
//set_csr(mstatus, MSTATUS_MIE);
//interrupts();
  // Clear out the interrupt pending bit, or the interrupt will 
  // just happen again as soon as we return,
  // and we'll never get back into our loop() function. 
  // These are write-one-to-clear.

  
 
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subroutine for reading the gyro
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void gyro_signalen(){
//noInterrupts();
     mpu.ReadRegs(MPUREG_GYRO_XOUT_H,response_gyro,6);
  for(int i = 1; i < 4; i++) {
        bit_data = ((int16_t)response_gyro[(i-1)*2]<<8)|response_gyro[(i-1)*2+1];
        MPUdata = (float)bit_data;
        gyro_axis[i] = MPUdata;
    }
   mpu.ReadRegs(MPUREG_ACCEL_XOUT_H,response_acc,6);
   for(int i = 1; i < 4; i++) {
        bit_data = ((int16_t)response_acc[(i-1)*2]<<8)|response_acc[(i-1)*2+1];
        MPUdata = (float)bit_data;
        acc_axis[i] = MPUdata;
    }

 if(cal_int == 2000){
    gyro_axis[1] -= gyro_axis_cal[1];                                       //Only compensate after the calibration.
    gyro_axis[2] -= gyro_axis_cal[2];                                       //Only compensate after the calibration.
    gyro_axis[3] -= gyro_axis_cal[3];                                       //Only compensate after the calibration.
  }
  gyro_roll = gyro_axis[eeprom_data[28] & 0b00000011];                      //Set gyro_roll to the correct axis that was stored in the EEPROM.
  if(eeprom_data[28] & 0b10000000)gyro_roll *= -1;                          //Invert gyro_roll if the MSB of EEPROM bit 28 is set.
  gyro_pitch = gyro_axis[eeprom_data[29] & 0b00000011];                     //Set gyro_pitch to the correct axis that was stored in the EEPROM.
  if(eeprom_data[29] & 0b10000000)gyro_pitch *= -1;                         //Invert gyro_pitch if the MSB of EEPROM bit 29 is set.
  gyro_yaw = gyro_axis[eeprom_data[30] & 0b00000011];                       //Set gyro_yaw to the correct axis that was stored in the EEPROM.
  if(eeprom_data[30] & 0b10000000)gyro_yaw *= -1;                           //Invert gyro_yaw if the MSB of EEPROM bit 30 is set.

  acc_x = acc_axis[eeprom_data[29] & 0b00000011];                           //Set acc_x to the correct axis that was stored in the EEPROM.
  if(eeprom_data[29] & 0b10000000)acc_x *= -1;                              //Invert acc_x if the MSB of EEPROM bit 29 is set.
  acc_y = acc_axis[eeprom_data[28] & 0b00000011];                           //Set acc_y to the correct axis that was stored in the EEPROM.
  if(eeprom_data[28] & 0b10000000)acc_y *= -1;                              //Invert acc_y if the MSB of EEPROM bit 28 is set.
  acc_z = acc_axis[eeprom_data[30] & 0b00000011];                           //Set acc_z to the correct axis that was stored in the EEPROM.
  if(eeprom_data[30] & 0b10000000)acc_z *= -1;                              //Invert acc_z if the MSB of EEPROM bit 30 is set.
//interrupts();
  
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subroutine for calculating pid outputs
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//The PID controllers are explained in part 5 of the YMFC-3D video session:
//https://youtu.be/JBvnB0279-Q 
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void calculate_pid(){
  //Roll calculations
 // noInterrupts();
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp;

  //Pitch calculations
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp;

  //Yaw calculations
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

  pid_last_yaw_d_error = pid_error_temp;
//  interrupts();
}

//This part converts the actual receiver signals to a standardized 1000 – 1500 – 2000 microsecond value.
//The stored data in the EEPROM is used.
  int32_t convert_receiver_channel(byte function){
  byte channel, reverse;                                                       //First we declare some local variables
 volatile int32_t low, center, high, actual;
 volatile  int32_t difference;

  channel = eeprom_data[function + 23] & 0b00000111;                           //What channel corresponds with the specific function
  if(eeprom_data[function + 23] & 0b10000000)reverse = 1;                      //Reverse channel when most significant bit is set
  else reverse = 0;                                                            //If the most significant is not set there is no reverse

  actual = receiver_input[channel];                                            //Read the actual receiver value for the corresponding function
  low = (eeprom_data[channel * 2 + 15] << 8) | eeprom_data[channel * 2 + 14];  //Store the low value for the specific receiver input channel
  center = (eeprom_data[channel * 2 - 1] << 8) | eeprom_data[channel * 2 - 2]; //Store the center value for the specific receiver input channel
  high = (eeprom_data[channel * 2 + 7] << 8) | eeprom_data[channel * 2 + 6];   //Store the high value for the specific receiver input channel


  if(actual < center){                                                         //The actual receiver value is lower than the center value
    if(actual < low)actual = low;                                              //Limit the lowest value to the value that was detected during setup
    difference = ((center - actual) * 500) / (center - low);                   //Calculate and scale the actual value to a 1000 - 2000us value
    if(reverse == 1)return 1500 + difference;                                  //If the channel is reversed
    else return 1500 - difference;                                             //If the channel is not reversed
  }
  else if(actual > center){                                                                        //The actual receiver value is higher than the center value
    if(actual > high)actual = high;                                            //Limit the lowest value to the value that was detected during setup
    difference = ((actual - center) * 500) / (high - center);                  //Calculate and scale the actual value to a 1000 - 2000us value
    if(reverse == 1)return 1500 - difference;                                  //If the channel is reversed
    else return 1500 + difference;                                             //If the channel is not reversed
  }
  else return 1500;

}

 
void set_gyro_registers(){ 
  
  mpu.init(false);
  uint8_t wai = mpu.whoami();
  if (wai == 0x71){
    Serial.println(">> MPU9250 Successful connection");
  }
  else{
    Serial.print(">> MPU9250 Failed connection: ");
    Serial.println(wai, HEX);
  }
  mpu.set_gyro_scale(BITS_FS_500DPS);
  mpu.set_acc_scale(BITS_FS_8G);
}
