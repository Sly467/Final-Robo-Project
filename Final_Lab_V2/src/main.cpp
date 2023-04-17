/*Cooper B, Samya P, Tyler S 

Basic Robot Movement with a PID feedback: 

This code's purpose is to use rotary encoders
(through the use of interupt functions) 
to move the robot set distances while constantly updating PWM
Speed using a PID feedback loop. DATE: 4/10/2023
*/
#include <Arduino.h>
#include <math.h>
#include <VL53L0X.h>
#include <Servo.h>
#include <Wire.h>


VL53L0X sensor; //Lidar sensor
uint16_t distance; // Stored distance

int buttonpin = 13; // define D0 Sensor Interface
bool val;// define numeric variables val
int flag = 0;

Servo gripper;
int pos = 0;
int openNum = 45;
int closeNum = 120;
const char gripPin = 9;

int temp_rw = 0;
int temp_lw = 0;

int dist_from_orig_RW = 0;
int dist_from_orig_LW = 0;

volatile unsigned int enc_LW; // LW Encoder tick counter
volatile unsigned int enc_RW; // RW Encoder tick counter

const int interruptPinLW = 2; // LW Encoder interrupt pinout
const int interruptPinRW = 3; // RW Encoder interrupt pinout

bool Start = true; //Flag that prevents the prevents the void loop from repeating

static int current_millis; // a means of resetting a timer

static int elapsed_LW; // time elapsed from LW current_millis set
static int elapsed_RW; // time elapsed from RW current_millis set

static int goal_tk_LW; // the expected encoder count of LW at a given time
static int goal_tk_RW; // the expected encoder count of RW at a given time

//initializing error values
static double curr_error_LW = 0;
static double curr_error_RW = 0;

static double prev_error_LW = 0;
static double prev_error_RW = 0;

static double prev2_error_LW = 0;
static double prev2_error_RW = 0;

//PID gain values lol
const double Kp = .2;
const double Ki = .001;
const double Kd = .00001;

//initalization of delta values used in PID
static double delta_LW = 0;
static double delta_RW = 0;

//initalization of PWM soeed
static int speedLW = 100;
static int speedRW = 100;

const double a = 39.3701; // side a of triangle in inches
const double b = 78.7402; // side b of triangle in inches

//double hyp_ticks = 24 * sqrt((a*a)+(b*b)); // hypotenuse distance in ticks equation using a and b rectangle
//double theta_rads = atan(a/b); // angle theta using sides a and b of rectangle
//double arc_ticks = theta_rads * 2.5 * 24; // arc_length distance in ticks using theta

// Motor A
 const int PWMA = 5;
 const int AIN1 = 32;
 const int AIN2 = 34;

  //Motor B
 const int PWMB = 4;
 const int BIN1 = 30;
 const int BIN2 = 36;

int stopCounter = 0; //counter in charge of changing opperation of the bot

bool dist_orig_check()
{
    if((enc_LW >= dist_from_orig_LW) || (enc_RW >= dist_from_orig_RW))
    return true;
    else
    return false;
}

void openGripper()
{
  for(pos = closeNum; pos >= openNum; pos -= 1){
    gripper.write(pos);
    delay(15);
  }
}

void closeGripper()
{
  for (pos = openNum; pos <= closeNum; pos += 1)
  {
    gripper.write(pos);
    delay(15);
  }
}

int PIDLW() // PID speed updater for LW
{
  elapsed_LW = millis() - current_millis; // New "reset Millis();
  
  if(elapsed_LW % 150 == 0){//takes a sample every 100 millisec
      
    goal_tk_LW = .262 * elapsed_LW; // Speed expected for wheel (.262/millisec)
    
    //Serial.print("goal_tick: ");
    //Serial.println(goal_tk_LW);
    
    prev2_error_LW = prev_error_LW; //sets 2nd to last error to previous error
    prev_error_LW  = curr_error_LW; //sets previous error to current error
    curr_error_LW = goal_tk_LW - enc_LW; // creates new current error
    
    // this value will be added to motor speed
    delta_LW = Kp * (curr_error_LW - prev_error_LW)   + Ki * (curr_error_LW + prev_error_LW)  + Kd * (curr_error_LW - 2* prev_error_LW + prev2_error_LW);  
    
    //Serial.print("delta_LW: ");
    //Serial.println(delta_LW);
    
    // removes any outliers
    if (delta_LW > 500)
    {
      delta_LW = 0;
    }
    
    else if (delta_LW < -500)
    {
      delta_LW = 0;
    }
    
    
    speedLW = speedLW + delta_LW; //adds delta to speed
    
    //Serial.print("speedLW: ");
    //Serial.println(speedLW);
    
    // speed bounding
    if (speedLW < 10)
    {
      speedLW = 10;
    }
    else if (speedLW > 200)
    {
      speedLW = 200;
    }
    
    return speedLW;//returns new motor speed value
  }
}

int PIDRW() // PID speed updater for LW
{
  
  elapsed_RW = millis() - current_millis;
  
  if(elapsed_RW % 150 == 0){//takes a sample every 100 millisec
    goal_tk_RW = .262 * elapsed_RW; // Speed expected for wheel (.262ticks/millisec)
    
    //Serial.print("goal_tick: ");
    //Serial.println(goal_tk_RW);
    
    prev2_error_RW = prev_error_RW; //sets 2nd to last error to previous error
    prev_error_RW  = curr_error_RW; //sets previous error to current error
    curr_error_RW = goal_tk_RW - enc_RW; // creates new current error
    
    //this value will be added to motor speed
    delta_RW = Kp * (curr_error_RW - prev_error_RW)   + Ki * (curr_error_RW + prev_error_RW)  + Kd * (curr_error_RW - 2* prev_error_RW + prev2_error_RW); 
    
    //Serial.print("delta_RW: ");
    //Serial.println(delta_RW);
    
    if (delta_RW > 500)
    {
      delta_RW = 0;
    }
    else if (delta_RW < -500)
    {
      delta_RW = 0;
    }
    
    speedRW = speedRW + delta_RW; //adds delta to speed
    
    //Serial.print("Speed: ");
    //Serial.println(speedRW);
    
    if (speedRW < 10)
    {
      speedRW = 10;
    }
    else if (speedRW > 200)
    {
      speedRW = 200;
    }
    
    return speedRW;//returns new motor speed value
  }
}

void LW() // LW Encoder Incrementer for Interrupt
{
  ++enc_LW;
}

void RW() // RW Encoder Incrementer for Interrupt
{
  ++enc_RW;
}

void setSpeed(int speed_LW, int speed_RW) //sets desired speed of each motor
{
    analogWrite(PWMA, speed_LW); //Sets the speed of motor
    analogWrite(PWMB, speed_RW); //Sets the speed of motor
}

void stopRobot() // stops both motors
{
  analogWrite(PWMA, 0); 
  analogWrite(PWMB, 0);
}

bool grab_dist_check() //distance to move forward to grab or reverse to let go
{
    if((enc_LW >= 38) || (enc_RW >= 38))
      return true;
    else
      return false;
  
}

void resetEncoder() //resets encoder values to 0
{
   enc_LW = 0;
   enc_RW = 0;
}

void resetValues() //resets error values to 0
{
  curr_error_LW = 0;
  curr_error_RW = 0;
  
  prev_error_LW = 0;
  prev_error_RW = 0;
  
  prev2_error_LW = 0;
  prev2_error_RW = 0;
}

bool halfBotRot()//checks to see if the motors have reached half a rotation
{
    if((enc_LW >= 224) || (enc_RW >= 224))
    return true;
    else
    return false;
}

void Forward() //sets both motors direction to allow bot to move forward
{
digitalWrite(AIN1,0);
digitalWrite(AIN2,1);
digitalWrite(BIN1,0);
digitalWrite(BIN2,1);
}
void Reverse() //sets both motors direction to allow bot to move forward
{
digitalWrite(AIN1,1);
digitalWrite(AIN2,0);
digitalWrite(BIN1,1);
digitalWrite(BIN2,0);
}

void Rotate_CCW() //sets both motors direction to allow bot to rotate counter clockwise
{
digitalWrite(AIN1,1);
digitalWrite(AIN2,0);
digitalWrite(BIN1,0);
digitalWrite(BIN2,1);
}

void Rotate_CW() //sets both motors direction to allow bot to rotate clockwise
{
digitalWrite(AIN1,0);
digitalWrite(AIN2,1);
digitalWrite(BIN1,1);
digitalWrite(BIN2,0);
}

void setup ()
{
gripper.attach(9);
Serial.begin(115200);
sensor.init();
sensor.startContinuous();
Wire.begin();
 pinMode (buttonpin, INPUT) ;// output interface D0 is defined sensor
 //attachInterrupt(digitalPinToInterrupt(3), a1, FALLING);
 pinMode (46, INPUT);
 pinMode (44, INPUT);
 gripper.write(140);
 pinMode(interruptPinLW, INPUT_PULLUP);
 pinMode(interruptPinRW, INPUT_PULLUP);
 attachInterrupt(digitalPinToInterrupt(interruptPinLW), LW, RISING);
 attachInterrupt(digitalPinToInterrupt(interruptPinRW), RW, RISING);

pinMode(PWMA, OUTPUT);
pinMode(AIN1, OUTPUT);
pinMode(AIN2, OUTPUT);

pinMode(PWMB, OUTPUT);
pinMode(BIN1, OUTPUT);
pinMode(BIN2, OUTPUT);
}

/*void loop()
{
    while (digitalRead(buttonpin)== LOW)
    {
      
    }
    if(flag == 0)
    {
      openGripper();
      flag++;
    }

    else if (flag == 1)
    {
      closeGripper();
      flag--;
    }
    

}
*/

void loop ()
{
 // digital interface will be assigned a value of pin 3 to read val
 
 while(digitalRead(buttonpin) == HIGH)//stops robot function, waits for clap or yell
 {
   stopRobot();
 }
 
 /*while(millis()%10 == 0)//stops robot function, waits for clap or yell
 {
   stopRobot();
   delay(10000);
 }*/
 
 if (flag == 0) //First Step
 {
  Serial.println(flag);
  Serial.println("high");
 //driver forward, detect object
  Forward();
  speedLW = 100;
  speedRW = 100;
  setSpeed(speedLW, speedRW);
  
  resetValues();
  resetEncoder();
  current_millis = millis();
  temp_rw = enc_RW;
  temp_lw = enc_LW;
     
  while( sensor.readRangeContinuousMillimeters() >= 200)
  {
      delay(20);
      Serial.println(sensor.readRangeContinuousMillimeters());
      delay(20);
      setSpeed(PIDLW(), PIDRW());
  }
  dist_from_orig_RW = enc_RW - temp_rw;
  dist_from_orig_LW = enc_LW - temp_lw;

 flag++;
 }
 
 else if (flag == 1) //Second Step
 {
  Serial.println(flag);
  Serial.println("high");
 //open gripper 
 openGripper();
 
 //move forward slightly
   Forward();
   speedLW = 100;
   speedRW = 100;
   setSpeed(speedLW, speedRW);
    
   resetValues();
   resetEncoder();
   current_millis = millis();
     
  while(grab_dist_check() == false)
  {
    Serial.println(enc_RW);
      setSpeed(PIDLW(), PIDRW());
  }
  stopRobot();
 //close gripper
closeGripper();

 flag++;
 }

 else if (flag == 2) // third step
 {
  Serial.println(flag);
  Serial.println("high");
 //turn around, move forward, detect object
  Rotate_CCW();
  speedLW = 100;
  speedRW = 100;
  setSpeed(speedLW, speedRW);
    
  resetValues();
  resetEncoder();    
  current_millis = millis();
     
  while(halfBotRot() == false)
  {
      setSpeed(PIDLW(), PIDRW());
  }
  stopRobot();
  delay(3000);

//move forward
  Forward();
  speedLW = 100;
  speedRW = 100;
  setSpeed(speedLW, speedRW);
    
  resetValues();
  resetEncoder();    
  current_millis = millis();
     
  while(dist_orig_check() == false)
  {
      setSpeed(PIDLW(), PIDRW());
  }
  dist_from_orig_LW = 0;
  dist_from_orig_RW = 0;

 flag++;
 }

else // final step and reset
{
  Serial.println(flag);
  Serial.println("high");
  //open gripper
  openGripper();
  //reverse short distance
  Reverse();
  speedLW = 100;
  speedRW = 100;
  setSpeed(speedLW, speedRW);
    
  resetValues();
  resetEncoder();    
  current_millis = millis();
     
  while(grab_dist_check() == false)
  {
      setSpeed(PIDLW(), PIDRW());
  }
  stopRobot();
  delay(3000);
  //turn around
  Rotate_CCW();
  speedLW = 100;
  speedRW = 100;
  setSpeed(speedLW, speedRW);
    
  resetValues();
  resetEncoder();    
  current_millis = millis();
     
  while(halfBotRot() == false)
  {
      setSpeed(PIDLW(), PIDRW());
  }
  stopRobot();
  delay(1000);
 //close gripper
  closeGripper();
  delay(3000);
flag = 0;
}

}