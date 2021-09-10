/*
Authors: Christopher LaBorde, Matthew Folsom, Caleb Scott, Holly Flynn, Henry Goff, Wesley Martin, Henry Salgado, Anna Hawcroft, Ashley Van Ausdale, Reva Fowler
Date: August 10th, 2021

This is a rewrite of the last Working Robot code complete with
-Threading
-Sensor Read function
-Move Function
-WiFi Connection


This code is for the robots that were built during Summer 2021 internship, with Lidar sensor and Adafruit powersensor.
Hardware libraries are called at beginning of program.

*/


// >>>>>>>>>>>>>>>>>>>>>>>
// Include Libraries
// >>>>>>>>>>>>>>>>>>>>>>>

// Wireless
#include <SPI.h>
#include <WiFi.h>

// Servo Motor
#include <ESP32Servo.h>

// Power Sensor
#include <Adafruit_INA260.h>

// Lidar Sensor
#include <ComponentObject.h>
#include <RangeSensor.h>
#include <SparkFun_VL53L1X.h>
#include <vl53l1x_class.h>
#include <vl53l1_error_codes.h>
#include <Wire.h>

// Threading Timer
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"

#define ONBOARD_LED 2

//>>>>>>>>>>>>>>>>>>>>>>>>>>>
//Declare Global Variables
//>>>>>>>>>>>>>>>>>>>>>>>>>>>

// Servo Parameters

// Set center values
int frontCenter = 90;
int rearCenter = 90;

// Set the angle at which the front/rear servo will deviate from center
int frontAngle = 10;
int rearAngle = 20;

// Set values for timing delay in ms between servo movements
int stepPhase = 200; // Will be used for timing between servo moves
int stepDelay = 100; // Can be another timing parameter. To be implemented 

// Initialize other useful variables to be used later on
int numSteps = 5; // number of steps per walk
int i;

bool isWalking = false; // At this point this variable is not used for anything
bool taskStarted = false;
bool running = true;
bool takeSamples = true;
bool trainSamples = false;
bool printSamples = false;
bool loopSteps = true;
bool onBoardTraining = true;

float powerIntegral = 0.0f;
float energyConsumed = 0.0f;
float energyOverDistance = 0.0f;
long powerSampleCount = 0L;
long distanceSampleCount = 0L;
int initialWalkDelay = 10000;
float preWalkDelay = 100.0f;
float postWalkDelay = 100.0f;
float interWalkDelay = 10000.0f;

float clientCheckDelay = 300.0f;
float distance = 0.0f;
float walkDistance = 0.0f;
float minDistance = 9999999;
float maxDistance = 0.0f;

int begTime;
int endTime;
int walkDuration;

float paramArray[3] = {frontAngle, rearAngle, stepPhase};

//>>>>>>>>>>>>>>>>>>>>>>>>
//Task Initializations
//>>>>>>>>>>>>>>>>>>>>>>>>

//Creates tasks to be used for multi-core
TaskHandle_t Sensor_reads;
TaskHandle_t Movement;


// >>>>>>>>>>>>>>>>>>>>>>>
// Wireless Initializations and Variables
// >>>>>>>>>>>>>>>>>>>>>>>

const int ServerPort = 23;
WiFiServer Server(ServerPort); // Sets up server
char ssid[] = "BLRP";
WiFiClient client; //Initializes client


// >>>>>>>>>>>>>>>>>>>>>>>
// Servos
// >>>>>>>>>>>>>>>>>>>>>>>
Servo rearServo;
Servo frontServo;

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// Hardware Pins
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// Current sensor
const int currentPin = 21;

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// Hardware Variables
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

// Current Sensor
int32_t rawValue = 0;
int R = 220;  // Ohms
float Vcc = 5.60;
float Current = 0;

Adafruit_INA260 ina260 = Adafruit_INA260();
struct INA260_Power_Struct{
  float Voltage;
  float Current;
  float Power;
};

float voltage;
float current;
float power;

float dataArray[100]; // TODO: Finalize size of array !!!!!!!!!!!!!!!!!!!!!!!!!!!!
int dataIndex = 0; 

//VL53L1 Lidar
SFEVL53L1X distanceSensor;


// Power Sensor read outs
INA260_Power_Struct read_INA260_Power()
{
  INA260_Power_Struct INA260_Power;
  INA260_Power.Current = ina260.readCurrent();
  INA260_Power.Voltage = ina260.readBusVoltage();
  INA260_Power.Power = ina260.readPower();
  return INA260_Power;
}

// LIDAR read outs
float read_VL53L1_Distance()
{
  distanceSensor.startRanging(); // Write configuration bytes to initieate measurement
  while (!distanceSensor.checkForDataReady())
    { // Check if data is ready from Lidar
      delay(500);
    }
  int distance = distanceSensor.getDistance(); // Save result of sensor reading
  distanceSensor.clearInterrupt();
  distanceSensor.stopRanging();

  return distance; //returns distance in mm
}


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// Servo Instructions
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

void moveForward()
{
  // As written:
  frontServo.write(frontCenter + frontAngle); // Lift Front Left Leg
  delay(stepPhase);
  rearServo.write(rearCenter - rearAngle); // Advance Rear Right Leg
  delay(stepPhase);
  frontServo.write(frontCenter - frontAngle); // Lift Front Right Leg
  delay(stepPhase);
  rearServo.write(rearCenter + rearAngle); // Advance Rear Left Leg 
  delay(stepPhase);
}



// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// Functions for multi-core 
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

// Create function to control movement
void Move(void * pvParameters)
{
  Serial.print("Move running on core: ");
  Serial.println(xPortGetCoreID());
  while(running)
  {
    takeSamples = true;
    delay(preWalkDelay);
    isWalking = true;
    Serial.println("Started Walking.");
    begTime = millis();
    for (int i = 0; i < numSteps; i++) // Set moveForward function to iterate over number of steps
    {
      Serial.print("Step ");
      Serial.println(i); // Will indicate location of step in program flow 
      moveForward();
    }
    delay(postWalkDelay);
    takeSamples = false;
    endTime = millis();
    walkDuration = endTime - begTime;
    printSamples = true;
    frontServo.write(frontCenter); // will return servos to center position after walk
    rearServo.write(rearCenter);
    isWalking = false;             // update isWalking status
    Serial.println("Stopped Walking");
    delay(interWalkDelay);

    if(!loopSteps)
      running = false;
  }
}


void mattSensorLoop (void * pvParameters)
{
  Serial.print("Sensor running on core ");
  Serial.println(xPortGetCoreID());
  Serial.print("DataIndex is: ");
  Serial.println(dataIndex);

  bool systemOK = true;
  if(!onBoardTraining)
  {
    while(Server.available() == 0) // loop added to continuously check for connection
    {
      systemOK = false;
      // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
      // Wireless Loop
      // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

      client = Server.available();  // Searches for a client connected to the server that has data available 
                          // for reading. If no such client exists (or there is no data available for reading)
                          // Server.available() will return "0". In this case, "Server" is ESP32,
      Serial.println();   // "client" is the python TCP script (I believe).
      Serial.println(client);
      if(client)
      {
        systemOK = true;
        Serial.println("New Client.");
        digitalWrite(ONBOARD_LED, HIGH);
      }
    }
  }

  if(systemOK)
  {
    while(running)
    {
      if(takeSamples)
      {
        powerSampleCount++;
        INA260_Power_Struct powerStruct = read_INA260_Power();
        powerIntegral = (1.0f/(float)powerSampleCount) * powerStruct.Power + (1 - (1.0f/(float)powerSampleCount)) * powerIntegral;
        
// update stored maximum and minimum values for distance
        distance = read_VL53L1_Distance();
        if(distance > maxDistance) 
          maxDistance = distance;
        if(distance < minDistance)
          minDistance = distance;
      }
      if(printSamples)
      {
        Serial.print("Total power integrated over walk (mW): ");
        Serial.println(powerIntegral);
        Serial.print("Distance Traveled (mm): ");
        walkDistance = maxDistance - minDistance;
        Serial.println(walkDistance);
        Serial.print("Distance Traveled (in): "); // Added inch conversion for verification
        Serial.println(walkDistance * 0.0394);
        
        Serial.print("Duration of Walk (mS): ");
        Serial.println(walkDuration);
        Serial.print("Energy Consumed (milliJoules): ");
        energyConsumed = powerIntegral * walkDuration / 1000;
        Serial.println(energyConsumed); // powerIntegral is cumulative sum of power readings (mW). 
                                        // WalkDuration is in milliseconds. So powerIntegral * walkDuration = mW * mS = microJoules. Divide by 1000 for millijoules
        energyOverDistance = energyConsumed / walkDistance;
        Serial.print("Avg Energy Spent Per Distance (millijoules/mm): ");
        Serial.println(energyOverDistance);        
        printSamples = false;


        dataArray[dataIndex] = energyOverDistance;
        dataIndex++;
        

        distance = 0;
        minDistance = 999999;
        maxDistance = 0;
        powerIntegral = 0;
        powerSampleCount = 0;
      }

      if(trainSamples && !onBoardTraining)
      {
        // send python values
         // Send the array of sensor data to python 
          client.write((uint8_t *)dataArray, sizeof(dataArray));
          Serial.println("We've Client.writed to the client (python)"); // Debugging statement
      }
      else if(trainSamples)
      {
        // *********************************************************
        // Train Here
        // *********************************************************
      }
    }
  }else
    {
     Serial.println("Wifi Connection Error System Exit!!!");
     running = false; 
    }
  }






void setup(){
  pinMode(ONBOARD_LED, OUTPUT);
  
  // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
  // Wireless setup
  // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
  Serial.begin(115200);
  Serial.print("SSID: ");
  Serial.println(ssid);

  WiFi.softAP(ssid);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP Address: ");
  Serial.println(myIP);
  Server.begin();

  Serial.println("Server Started");

  
  // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
  // Hardware setup
  // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

  //Power Sensor
  Serial.println("Adafruit INA260 Test");
  if(!ina260.begin())
  {
    Serial.println("Couldn't Find INA260 Chip");
    while (1); //      !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  }else{
    Serial.println("Adafruit INA260 Found");
       }

  //Lidar
  Wire.begin();
  Serial.println("VL53L1X Qwiic Test");
  if(distanceSensor.begin() != 0) //Begin returns 0 on a good init
  {
    Serial.println("Lidar Sensor failed to begin. Freezing. . . ");
  }
  else
  {
    Serial.println("VL53L1X Found");
    Serial.println("Lidar Sensor online!");
  }

  // Initialize Servos
  frontServo.attach(2);
  rearServo.attach(4);
  frontServo.write(frontCenter);
  rearServo.write(rearCenter);
  delay(initialWalkDelay);


  if(!taskStarted)
  {
    taskStarted = true;
    
    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    // Threading Setup
    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    
    xTaskCreatePinnedToCore(
      Move,
      "Move",
      10000,
      NULL,
      1,
      &Movement,
      1);

    xTaskCreatePinnedToCore(
      mattSensorLoop, //Task Function
      "Sensors", //Task Name
      10000, //Stack Size
      NULL, //Parameter of the task
      1, //Priority of the task
      &Sensor_reads, // Task Handle to keep track of created task
      0); //Task pinned to core 0
  }
}



void loop(){
TIMERG0.wdt_wprotect=TIMG_WDT_WKEY_VALUE; // "TIMERGO functions prevent a watchdog error from being thrown due to error due to multi-core
TIMERG0.wdt_feed=1;
TIMERG0.wdt_wprotect=0;
}