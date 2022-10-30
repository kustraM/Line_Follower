#include <analogWrite.h>
#include <QTRSensors.h>
#include "CytronMotorDriver.h"
#include "BluetoothSerial.h"


//on-line and off-filne
#define limit 3000           //detection limit for sensors (should be ~3100)
#define soft_limit 1500     //detection limit for C5 due to partially broken sensor
int value=50;

//QTR-MD-06A
#define NUM_SENSORS             6       // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  10       // average 4 analog samples per sensor reading (maybe higher value? expreiments needed)
#define EMITTER_PIN             16      // emitter is controlled by digital pin IO16

// initialize an instance of bluetoothSerial for communication
BluetoothSerial ESP_BT;

//receiving variables
int speed = 0;      //maximum speed value
int gain = 0;       //prepared value for later PID controller

//id of used input
int id = -1;            //id is used to know where to write the read value (range 129-255)
int val_byte1 = -1;     //first byte for value (range 0-128)
int val_byte2 = -1;     //second byte for value (range 0-128)
//val_byte1 and val_byte2 will be combined to create a value from 0 to 16 384 (should be enough)

// variable for data input
int incoming;

// creating an instance of QTRSensorsAnalog to allow for the reading of the values from the sensors
QTRSensorsAnalog qtra((unsigned char[]) {36, 39, 34, 35, 15, 14}, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];
unsigned long previousMillis = 0;    
const long interval =10;  

//motors
/*for this we use the PWM_DIR mode of the CytronMD. One pin is for setting the PWM value (0-255)
 *and the second is for direction*/
CytronMD motorLeft(PWM_PWM, 13, 5);  // PWM 1 = Pin 5, DIR 2 =Pin 2.       1, 9
CytronMD motorRight(PWM_PWM, 4, 2); // PWM 2 = Pin 13, DIR 2 = Pin 27.   4,31

void setup() {
  Serial.begin(115200);   //initialize serial communication
  ESP_BT.begin("ESP32_Control");    //Activates the bluetooth module with the name ESP32_Control for everyone to see in the network
}

void loop() {
unsigned long currentMillis = millis();
if (currentMillis - previousMillis >= interval) {
previousMillis = currentMillis;
qtra.read(sensorValues);    //read all analogs

//this loop is used for printing out the read values from the sensors
for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');                           // tab to format the raw data into columns in the Serial monitor
  }
  Serial.println();

//depending on the combination set the proper motor speed
if((Limit_sensor(0) == 0 && Limit_sensor(1) == 0 && Limit_sensor(2) == 1 && Limit_sensor(3)==1 && Limit_sensor_4(4)==0 && Limit_sensor(5)==0) ||  
(Limit_sensor(0) == 1 && Limit_sensor(1) == 1 && Limit_sensor(2) == 1 && Limit_sensor(3)==1 && Limit_sensor_4(4)==1 && Limit_sensor(5)==1)){
  motorLeft.setSpeed(-50);
  motorRight.setSpeed(50);
}

else if ((Limit_sensor(0) == 0 && Limit_sensor(1) == 1 && Limit_sensor(2) == 1 && Limit_sensor(3)==0 && Limit_sensor_4(4)==0 && Limit_sensor(5)==0) || 
(Limit_sensor(0) == 1 && Limit_sensor(1) == 1 && Limit_sensor(2) == 1 && Limit_sensor(3)==0 && Limit_sensor_4(4)==0 && Limit_sensor(5)==0)){
  motorLeft.setSpeed(-25);
  motorRight.setSpeed(50);
}

else if ((Limit_sensor(0) == 0 && Limit_sensor(1) == 0 && Limit_sensor(2) == 0 && Limit_sensor(3)==1 && Limit_sensor_4(4)==1 && Limit_sensor(5)==0) ||  
(Limit_sensor(0) == 0 && Limit_sensor(1) == 0 && Limit_sensor(2) == 0 && Limit_sensor(3)==1 && Limit_sensor_4(4)==1 && Limit_sensor(5)==1)){
  motorLeft.setSpeed(-50);
  motorRight.setSpeed(25);
}
  
  
else if ((Limit_sensor(0) == 1 && Limit_sensor(1) == 1 && Limit_sensor(2) == 0 && Limit_sensor(3)==0 && Limit_sensor_4(4)==0 && Limit_sensor(5)==0)||
(Limit_sensor(0) == 1 && Limit_sensor(1) == 0 && Limit_sensor(2) == 0 && Limit_sensor(3)==0 && Limit_sensor_4(4)==0 && Limit_sensor(5)==0)){
  motorLeft.setSpeed(100);  
  motorRight.setSpeed(100);
}

else if ((Limit_sensor(0) == 0 && Limit_sensor(1) == 0 && Limit_sensor(2) == 0 && Limit_sensor(3)==0 && Limit_sensor_4(4)==1 && Limit_sensor(5)==1)||
(Limit_sensor(0) == 0 && Limit_sensor(1) == 0 && Limit_sensor(2) == 0 && Limit_sensor(3)==0 && Limit_sensor_4(4)==0 && Limit_sensor(5)==1)){
  motorLeft.setSpeed(-65);
  motorRight.setSpeed(-65); 
}

else if (Limit_sensor(0) == 0 && Limit_sensor(1) == 0 && Limit_sensor(2) == 1 && Limit_sensor(3)==1 && Limit_sensor_4(4)==1 && Limit_sensor(5)==1){
  motorLeft.setSpeed(-100);
  motorRight.setSpeed(-100); 
}

else if (Limit_sensor(0) == 1 && Limit_sensor(1) == 1 && Limit_sensor(2) == 1 && Limit_sensor(3)==1 && Limit_sensor_4(4)==0 && Limit_sensor(5)==0){
  motorLeft.setSpeed(100);
  motorRight.setSpeed(100); 
}


//BLE communication
//first check if Bluetooth connection is available
if (ESP_BT.available())
  {

    //read the incomming byte
    incoming = ESP_BT.read();

    //connect everything together
    if(incoming>127)
    {
      reset_rx_BT();
      id=incoming-128;
    }
    else if (val_byte1 == -1)
    {
      val_byte1 = incoming;
    }
    else if (val_byte2 == -1)
    {
      val_byte2 = incoming;
      //if everything was succesfull combine the 2 val_byte to create one value
      /* depending on the id assign the calculated value to the correct variable 
       * for now its only printed out*/
      int value = 128*val_byte1 + val_byte2;

      Serial.print("Id: "); Serial.print(id); Serial.print(", val: "); Serial.println(value);
      reset_rx_BT();
    }
  }
//delay(10);
}
}

//function to check if a sensor is exceeding a limit at which point it is read as a black line detection
boolean Limit_sensor(uint8_t num){
  if (sensorValues[num]>limit)
    return 1;
  else
    return 0;
}


//function to check if a sensor is exceeding a limit at which point it is read as a black line detection; only for sensor 4 (C5)
boolean Limit_sensor_4(uint8_t num){
  if (sensorValues[num]>soft_limit)
    return 1;
  else
    return 0;
}

void reset_rx_BT() {                              // function to erase all bytes (set to -1)
  id = -1;
  val_byte1 = -1;
  val_byte2 = -1;
}
