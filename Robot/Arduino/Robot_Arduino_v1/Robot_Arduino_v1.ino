/***********************************************************************
 * Robot Platooning Project - Arduino Inferrence 
 * Authur: Ayo Ayibiowu & Abhilash Hashyap
 * Contact: charlesayibiowu@hotmail.com
 *
 *This Programs Communicates with the Raspberry Pi or Jetson Nano.
 *It sends the current motor readins, sensors readings and other readings expected all using UART.
 *
 * It also have a lane following function implemented - The Lane following code 
 * makes the robot to stay on a lane. It uses the IR sensors to detect when the robot is appraching 
 * a black tape (lane seperation) and adjust the robot to fall back to the main lane/road.
 *
 *
 * v1
 * 
 * Changes and Features:
 *  - Send out sensor, motor, and other readings
 *  - Read motor commands, and other related commands from the controller
 *  - Implemented support for making robot to go straigth when the speed values are equal
 *  - Implement lane following : Robots follows the lane accordingly.
 * 
 * 
 ***********************************************************************/

#include <RedBot.h>
#include <Servo.h>

RedBotSensor left = RedBotSensor(A3);   // initialize a left sensor object on A3
RedBotSensor center = RedBotSensor(A6); // initialize a center sensor object on A6
RedBotSensor right = RedBotSensor(A7);  // initialize a right sensor object on A7
RedBotSensor extremeright = RedBotSensor(A5);
RedBotSensor extremeleft = RedBotSensor(A0);
Servo ServoClaws;


RedBotEncoder encoder = RedBotEncoder(A2, A4);  // initializes encoder on pins A2 and 10
int buttonPin = 12;
int countsPerRev = 192;   // 4 pairs of N-S x 48:1 gearbox = 192 ticks per wheel rev
float wheelDiam = 2.56;  // diam = 65mm / 25.4 mm/in
float wheelCirc = PI*wheelDiam;  // Redbot wheel circumference = pi*D
// variables used to store the left and right encoder counts.
long lCount;
long rCount;

// constants that are used in the code. LINETHRESHOLD is the level to detect 
// if the sensor is on the line or not. If the sensor value is greater than this
// the sensor is above a DARK line.
//
// SPEED sets the nominal speed

#define LINETHRESHOLD 800
#define SPEED 80 // sets the nominal speed. Set to any number from 0 - 255.

const float reduce_factor = 0.35;

RedBotMotors vehicle;
int leftSpeed;   // variable used to store the leftMotor speed
int rightSpeed;  // variable used to store the rightMotor speed

String inputVal, outputPacket;
int ind1,ind2,ind3,ind4,ind5,ind6,ind7;

int motorEnable,io_device1,io_device2,check1,check2;

int button=1;
int stat=0;
String aa,bb,cc,dd,ee,lcnt,rcnt,SMotorEnable,Sio1,Sio2;

void setup()
{   
    Serial.begin(57600);
    pinMode(buttonPin, INPUT_PULLUP);
    ServoClaws.attach(10);
    ServoClaws.write(0);

    // Initialize all variables first
    // the expected data packet
    inputVal = "0,0,0,0,0,0,0*";
    outputPacket = "0,0,0,0,0,0,0*";
}

void loop()
{

    // Check if data was available
    if (digitalRead(buttonPin) == LOW)
  {
    encoder.clearEnc(BOTH); }


    
    if (Serial.available() > 0) {
        
        inputVal = Serial.readStringUntil('\n'); // read the incoming byte:

        // Extract the needed data
        ind1 = inputVal.indexOf(',');
        
        ind2=inputVal.indexOf(',',ind1+1);

        ind3=inputVal.indexOf(',',ind2+1);

        ind4=inputVal.indexOf(',',ind3+1);

        ind5=inputVal.indexOf(',',ind4+1);

        ind6=inputVal.indexOf(',',ind5+1);

        ind7=inputVal.indexOf('*');
        

        check1 = (inputVal.substring(0,ind1)).toInt();
        
        check2 = (inputVal.substring(ind6+1,ind7)).toInt();

        if(check1==check2)
        {
          leftSpeed = -1 * ((inputVal.substring(ind1+1,ind2)).toInt());
          rightSpeed = (inputVal.substring(ind2+1,ind3)).toInt();
          motorEnable = (inputVal.substring(ind3+1,ind4)).toInt();
          io_device1 = (inputVal.substring(ind4+1,ind5)).toInt();
          io_device2 = (inputVal.substring(ind5+1,ind6)).toInt(); 
        }     

    }

   moveServoMotor();

   // update the motor speed
   updateMotorSpeed();
   
   // get the latest encoder values
   fetchEncoderValues();

   // get the latest ir values
   fetch_IR_values();

   // fetch and update others

   // send out the latest values to the host
   sendUpdate();

   // wait for a while
   delay(5);
   
      

}


void fetch_IR_values()
{
  // get all ir sensor values
   int a=(left.read());
   aa=String(a);
   int b=(center.read());
   bb=String(b);
   int c=(right.read());
   cc=String(c);
   int d=(extremeright.read());
   dd=String(d);
   int e=(extremeleft.read());
   ee=String(e);
}


void fetchEncoderValues()
{
  //get encoder values
   lCount = encoder.getTicks(LEFT);   
   rCount = encoder.getTicks(RIGHT);
   lcnt= String(lCount);
   rcnt= String(rCount);


}

void updateMotorSpeed()
{
  
  // updates the speed of the motor
  // check if motor enable is 0 or 1
  if (io_device2 == 99)
  {
    lanefollowing(leftSpeed, rightSpeed);    
  }
  else
  {
    
  if (motorEnable == 1)
  {

      if((-1*leftSpeed)==(rightSpeed))
      {
        driveStraight(rightSpeed);
        
        }
        else
    {
        vehicle.leftMotor(leftSpeed);
        vehicle.rightMotor(rightSpeed);
    }
    }

  else
  {
    // disable the motor
        vehicle.leftMotor(0);
        vehicle.rightMotor(0);
  }
  }
  
}

void sendUpdate()
{
  SMotorEnable= String(motorEnable);
  Sio1=String(io_device1);
  Sio2=String(io_device2);

  String toSend=ee+','+aa+','+ bb+','+ cc+','+dd+','+ lcnt+','+ rcnt+','+SMotorEnable+','+Sio1+','+Sio2+'\n';
  Serial.print(toSend);
  
  }
void driveStraight( int motorPower)
{
    
      // variables for tracking the left and right encoder counts
      long prevlCount, prevrCount;
    
      long lDiff, rDiff;  // diff between current encoder count and previous count
    
      // variables for setting left and right motor power
      //int leftPower =  -1*motorPower;
      //int rightPower = motorPower;
      int leftPower =-1*motorPower;
      int rightPower=motorPower;
      if(leftPower>0){
          
          leftPower=leftPower+2;
        }
        else
{
  
  leftPower=leftPower-2;
  }
      
    
      // variable used to offset motor power on right vs left to keep straight.
      int offset = 2;  // offset amount to compensate Right vs. Left drive
            lCount = encoder.getTicks(LEFT);
        rCount = encoder.getTicks(RIGHT);
       // vehicle.drive(motorPower);
             vehicle.rightMotor(rightPower);
        vehicle.leftMotor(leftPower);
    
        // calculate the rotation "speed" as a difference in the count from previous cycle.
        lDiff = (lCount - prevlCount);
        rDiff = (rCount - prevrCount);
    
        // store the current count as the "previous" count for the next cycle.
        prevlCount = lCount;
        prevrCount = rCount;
    
        // if left is faster than the right, slow down the left / speed up right
        if ((lDiff > rDiff) and (lDiff>0 or rDiff>0)) 
        {
          leftPower = leftPower - offset;
          rightPower = rightPower + offset;
        }
        // if right is faster than the left, speed up the left / slow down right
        else if ((lDiff < rDiff) and (lDiff>0 or rDiff>0)) 
        {
          leftPower = leftPower + offset;  
          rightPower = rightPower - offset;
        }
        else if ((lDiff < rDiff) and (lDiff<0 or rDiff<0)) 
        {
          leftPower = leftPower - offset;
          rightPower = rightPower + offset;
        }
        // if right is faster than the left, speed up the left / slow down right
        else if ((lDiff > rDiff) and (lDiff<0 or rDiff<0)) 
        {
          leftPower = leftPower + offset;  
          rightPower = rightPower - offset;
        }
  
        
     //   delay(50);
     
}

void moveServoMotor(){
  if(io_device1==150){
  ServoClaws.write(0);
  }
  else  if(io_device1==300)
  {
  ServoClaws.write(78);
  }
  
  }

  void lanefollowing(int speed_left, int speed_right)
  {
    if (extremeleft.read() > LINETHRESHOLD){
        vehicle.leftMotor(speed_left);
        vehicle.rightMotor(int(reduce_factor*speed_right));  
        delay(0.2); 
    }

    else if (extremeright.read() > LINETHRESHOLD){
        vehicle.rightMotor(speed_right);
        vehicle.leftMotor(int(reduce_factor*speed_left));  
        delay(0.2);
    }

    
    else if ((left.read() > LINETHRESHOLD) && (right.read() > LINETHRESHOLD) && (center.read() > LINETHRESHOLD))
    {
      vehicle.stop();
    }

    else if (right.read() > LINETHRESHOLD){
        vehicle.rightMotor(speed_right);
        vehicle.leftMotor(0);  
        delay(0.5);
    }

    else if (left.read() > LINETHRESHOLD){
        vehicle.rightMotor(0);
        vehicle.leftMotor(speed_left);  
        delay(0.5);
    }
    else
    {
       driveStraight(speed_right);
       }
  }
