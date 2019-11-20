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
 * v2 and v3 (failed version)
 * 
 *  - Provides support for calculating odometry on board  - NOT WORKING PROPERLY
 *  
 * v4 - 
 * 
 *  - Migrates all communication through ROS. The Arduino now acts as a node on the ROS network.
 *  - All received data has formatted into a readable struch packet
 *  - Code is more efficent and scalable
 *  - Readability has been improved
 *  - Odometry code removed. 
 *  - Support for Ultrasonic sensor: Sensor is used for detecting obstacle in front
 *  - Noise filter added to Ultrasonic
 *  
 * v5 -
 * 
 *  - Suport for lane changed implemented
 *  - Uses timer2 overflow interrupt to enable ROS Communication
 *  
 ***********************************************************************/


#include <ros.h>
#include <std_msgs/String.h>
#include <RedBot.h>
#include <Servo.h>

#include <ros.h>
#include <std_msgs/String.h>
//#include <TimerOne.h>


// Creates the instance for all the irSensors used
RedBotSensor left = RedBotSensor(A3);   // initialize a left sensor object on A3
RedBotSensor center = RedBotSensor(A6); // initialize a center sensor object on A6
RedBotSensor right = RedBotSensor(A7);  // initialize a right sensor object on A7
RedBotSensor extremeright = RedBotSensor(A5);
RedBotSensor extremeleft = RedBotSensor(A0);
Servo ServoClaws;

// variables encoders instance
RedBotEncoder encoder = RedBotEncoder(A2, A4);  // initializes encoder on pins A2 and 10

// Creates the instance for the motor
RedBotMotors vehicle;

//////////////////////////////////////////////////////////////////////////////////////
#define buttonPin 12

/****** Sensors Variables needed ********************/
// Variables to be sent out.
// Infrared red values
unsigned int extremeleft_ir = 0;
unsigned int left_ir = 0;
unsigned int center_ir = 0;
unsigned int right_ir = 0;
unsigned int extremeright_ir = 0;

// encoder values
long encoderLeft = 0;
long encoderRight = 0;

// motor status
byte motorEnable = 0;

// extra IOs to be used.
int extra_io1 = 0;
int extra_io2 = 0;

//holds the ultrasonic distance variable
unsigned int distance = 0;

// this variable will hold the full data packet
String dataOut = String(extremeleft_ir)+','+String(left_ir)+','+ String(center_ir)+','+ String(right_ir)+','+String(extremeright_ir)+','+ String(encoderLeft)+','+ String(encoderRight)+','+String(motorEnable)+','+String(extra_io1)+','+String(extra_io2)+','+String(distance);

// Creates a Struct to hold all the possible data coming from the Master
struct Robot{
  int leftSpeed;
  int rightSpeed;
  int motorEnable;
  int io_device1;
  int io_device2;
  int io_device3;
};

// creates an object for the struch that can be accessed global.
// This will store all the current robot state
Robot controller = {0, 0, 0, 0, 0, 0};

// Threshold for controlling the lane movemement
#define LINETHRESHOLD 800

const float reduce_factor = 0.35;

/****** Setup ROS HERE *************************************/

// Creates the ros instance
ros::NodeHandle  nh;

// Setup the ROS publisher
std_msgs::String str_msg;
ros::Publisher node("BumbleBee_Arduino", &str_msg);

// This functions handles the ROS message recieved and forwards 
// and updates the necessary variables
void ROS_reciever( const std_msgs::String& incoming){
  // recieved message will in the incoming.data
  // creates a temporary buffer to hold the incoming data
  const String dataIn = incoming.data;
  // since the data sent are comma seperated, we can read them easily.
  // create some temporary values to hold the incoming data
  int temp1, temp2, temp3, temp4, temp5, temp6, temp7, temp8;

  if (sscanf(dataIn.c_str(), "%d,%d,%d,%d,%d,%d,%d,%d", &temp1, &temp2, &temp3, &temp4, &temp5, &temp6, &temp7, &temp8) != 1)
  {
    // data read successfully.    
    // here we perform our data integrity check
    if (temp1 == temp8){
      // data integrity is correct.
      // then we can ready our data properly
      controller.leftSpeed = (-1 * temp2);
      controller.rightSpeed = temp3;
      controller.motorEnable = temp4;
      controller.io_device1 = temp5;
      controller.io_device2 = temp6;
      controller.io_device3 = temp7;
    }
  }

  // uncomment this code to test out if you are receving the proper data
  /*
  // test if data read was correct. publish data back
  const String temp_out = String(controller.leftSpeed) +','+controller.rightSpeed+','+String(controller.motorEnable)+','+String(controller.io_device1) +','+controller.io_device2+','+String(controller.io_device3);
  
  // lets attempt to publish this new temporary data back
  const char *tab3 = temp_out.c_str();
  str_msg.data = tab3;
  node.publish( &str_msg );
  */
}

// setup the ROS Subscriber
ros::Subscriber<std_msgs::String> sub("BumbleBee_Receiver", ROS_reciever );

/***** Odometry Related Variables ********************/
// Variable for holding position data
double ticks_to_mm = 1.078167;
int scanner_displacement = 0;
int robot_width = 160;

// position variables
struct Position {
  double x;
  double y;
  double theta;
};

// define the initial robot position
Position robot_state = {0, 0, 0};

// ultrasonic configuration
const byte triggerPin = 9;
const byte echoPin = 3;

// track if it's the first time the program is running
bool firstStart = true;

int pre_distance = 0;
int change_counter = 0;

int robot_connection = 0;

// lane change related variables
int changed=0;
int intersect=0;


void setup()
{   
    // Setup ROS first
    nh.initNode();
    nh.advertise(node);
    nh.subscribe(sub);
  
    pinMode(buttonPin, INPUT_PULLUP);
    
    // Attach the Servo to pin 10
    ServoClaws.attach(10);
    ServoClaws.write(0);

    // Configure the Ultrasonic sensor pins    
    pinMode(triggerPin, OUTPUT);
    pinMode(echoPin, INPUT);


    firstStart = true;
    change_counter = 0;

    /*
    // fetch the initial ticks
    pre_ticks[0] = encoder.getTicks(LEFT);   
    pre_ticks[1] = encoder.getTicks(RIGHT);
    */
    // setup up the timer interrupt
 //Timer1.initialize(5000); // 5000 microseconds - 0.05secs
  //Timer1.attachInterrupt( timerIsr);

    // End of setup
    nh.spinOnce();   

}


void loop()
{

    // Check if data was available
   if (digitalRead(buttonPin) == LOW)
      {
        encoder.clearEnc(BOTH);
        // also reset current position values
      
        // reset to initial state value
        robot_state.x = 0;
        robot_state.y = 0;
        robot_state.theta = 0;
        
        }
   
   // Updates the Servo motor
   moveServoMotor();

   // update the motor speed
   updateMotorSpeed();

   // send out the latest values to the host
   publish();

   // check if device is still connected to the pc
   robot_connection = nh.connected();
   // spinOnce needs to run as fast as possible, to ensure it can be able to handle data coming in as soon as possible
   nh.spinOnce();   
   // wait for a while
   delay(1);
}

void timerIsr()
{
    // call spin once
   // STATE = !STATE;
   nh.spinOnce();

}


void fetch_IR_values()
{
  // get all ir sensor values
   left_ir = left.read();
   center_ir = center.read();
   right_ir = right.read();
   extremeright_ir =(extremeright.read());
   extremeleft_ir = (extremeleft.read());
}

void fetchEncoderValues()
{
  //get encoder values
   encoderLeft = encoder.getTicks(LEFT);   
   encoderRight = encoder.getTicks(RIGHT);

}

void fetchUltrasonic()
{
  // Function for retreiving the distance

  long duration, cm;
  int change = 0;
    // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(triggerPin, LOW);

    // to the reception of its echo off of an object.
  duration = pulseIn(echoPin, HIGH);
  // convert the time into a distance
  cm = duration / 29 / 2;

  // here we are going to filter the cm results.
  // discard the outlier result that occasionally comes from the ultrasonic
  if (cm <= 550)
  {
    //distance = cm;
    if (firstStart == true)
    {
      pre_distance = cm;
      distance = cm;
      firstStart = false;
    }
      // also needs to filter our the result from the ultrasonic sensor.
  // for example if you have 50, the result can't be allowed to jump to 8 or 100

      else
      {
        change = abs(cm - pre_distance);
        if (change <= 20){
              distance = cm;
        }
        else
        {
          // we can send the previous distance or discard the result entirely
          change_counter ++;
          distance = pre_distance;
        }

        // Also don't forget to reset the change_counter so we don't end 
        // up with the old data always
        if (change_counter > 3)
        {
          change_counter = 0;
          distance = cm;
          firstStart = true;
        }
      }

      // update the previous distance
      pre_distance = distance;
  }
  
}
void updateMotorSpeed()
{
  
  // updates the speed of the motor
  // Check if the given command is for auto lane following
  if (controller.io_device2 == 99)
  {
    lanefollowing(controller.leftSpeed, controller.rightSpeed);
     if (controller.io_device3==1 && changed == 0)
    {
        LaneChangeLeft();
        controller.io_device3=0;
        changed=1;
      }
     if(controller.io_device3==2 && changed == 1)
     {
        LaneChangeRight();
        controller.io_device3=0;
        changed=0;
      }
      // intersection changing
      //////////////////////////////////////////////////////////////////
       if (controller.io_device3==3 && intersect == 0 )
    {
        InterEnterLeft();
        controller.io_device3=0;
        intersect=1;
      }
     if(controller.io_device3==4 && intersect == 0 )
     {
        InterEnterRight();
        controller.io_device3=0;
        intersect=1;
      }  
      /////////////////////////////////////////////////////////////////////
      if (controller.io_device3==5 and intersect == 1 )
    {
        InterExitLeft();
        controller.io_device3=0;
        intersect=0;
      }
     if(controller.io_device3==6 and intersect == 1 )
     {
        InterExitRight();
        controller.io_device3=0;
        intersect=0;
      }      
  }
  
  else
  {
    // Checks the status of the Motor enable      
    if (controller.motorEnable == 1)
    {  
      // If both speed values are same, then activate straight drive
        if((-1*controller.leftSpeed)==(controller.rightSpeed))
        {
          driveStraight(controller.rightSpeed);
          
          }
          
        else
          {
            vehicle.leftMotor(controller.leftSpeed);
            vehicle.rightMotor(controller.rightSpeed);
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
  dataOut = String(extremeleft_ir)+','+String(left_ir)+','+ String(center_ir)+','+ String(right_ir)+','+String(extremeright_ir)+','+ String(encoderLeft)+','+ String(encoderRight)+','+String(motorEnable)+','+String(extra_io1)+','+String(extra_io2)+','+String(distance)+','+String(robot_connection);
  // creates the temp array for converting String to char * or char array
  const char *temp_data = dataOut.c_str();
  str_msg.data = temp_data;
  node.publish( &str_msg );
  
  }

void publish()
{
   //fetch ultrasonic sensor
   fetchUltrasonic();
   // get the latest encoder values
   fetchEncoderValues();
   // get the latest ir values
    fetch_IR_values();
   // send out the latest values to the host
   sendUpdate();
   
}

void mydelay(int count = 1){
     unsigned int i = 0;
     for (i=0; i<count; i++)
     {
     //publish();
     nh.spinOnce();
     delay(1);
     }

}
  
void driveStraight( int motorPower)
{
    
      // variables for tracking the left and right encoder counts
      long prevlCount, prevrCount, lCount, rCount;
    
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
       
}

void moveServoMotor()
{
  if(controller.io_device1==150)
    {
      ServoClaws.write(0);
    }
  else  if(controller.io_device1==300)
    {
      ServoClaws.write(78);
    }
}

void lanefollowing(int speed_left, int speed_right)
{
  /// loging purpose - my speed value doesn't change.
  //const String speedValue = "Lane following code: "+String(speed_left) + " : " + String(speed_right);

   
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

void LaneChangeLeft()
{

        vehicle.leftMotor(70);
        vehicle.rightMotor(70);
        mydelay(1000);
        vehicle.drive(70);
        mydelay(1500);
        vehicle.leftMotor(-70);
        vehicle.rightMotor(-70);
        mydelay(1000);
  
  
}

void LaneChangeRight()
{
 
        vehicle.leftMotor(-70);
        vehicle.rightMotor(-70);
        mydelay(1000);
        vehicle.drive(70);
        mydelay(1500);
        vehicle.leftMotor(70);
        vehicle.rightMotor(70);
        mydelay(1000);
        
}


void InterEnterLeft()
{
  vehicle.leftMotor(70);
  vehicle.rightMotor(70);
  delay(800);
  vehicle.drive(70);
  delay(500);
  }
void InterEnterRight()
{
  vehicle.leftMotor(-70);
  vehicle.rightMotor(-70);
  delay(800);
  vehicle.drive(70);
  delay(500);
  }
void InterExitLeft()
{ 
  vehicle.drive(70);
  delay(1000);
  vehicle.leftMotor(70);
  vehicle.rightMotor(70);
  delay(1000);
  }
void InterExitRight()
{ 
  vehicle.drive(70);
  delay(1000);
  vehicle.leftMotor(-70);
  vehicle.rightMotor(-70);
  delay(1000);
  } 
