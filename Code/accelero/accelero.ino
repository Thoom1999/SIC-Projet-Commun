#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SPI.h>
#include <SD.h>
// #include <utility/imumaths.h>

/* This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
   which provides a common 'type' for sensor data and some helper functions.

   To use this driver you will also need to download the Adafruit_Sensor
   library and include it in your libraries folder.

   You should also assign a unique ID to this sensor for use with
   the Adafruit Sensor API so that you can identify this particular
   sensor in any data logs, etc.  To assign a unique ID, simply
   provide an appropriate value in the constructor below (12345
   is used by default in this example).

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3-5V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 500;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

File myFile;


void setup(void)
{
  Serial.begin(9600);

  while (!Serial) delay(10);  // wait for serial port to open!

  Serial.println(F("Orientation Sensor Test")); Serial.println(F(""));

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print(F("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!"));
    while (1);
  }

  delay(2000);


  /* Initialise the SD card */
  Serial.print(F("Initializing SD card..."));

  if (!SD.begin(4)) {
    Serial.println(F("initialization failed!"));
    while (1);
  }
  Serial.println(F("initialization done."));
  delay(1000);
}

void loop(void)
{
  //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
  // sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;

  myFile = SD.open("test.txt", FILE_WRITE);

  sensors_event_t orientationData, accelerometerData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    sensors_event_t *a = &accelerometerData;
    sensors_event_t *o = &orientationData;
    
    double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  
    myFile.print(F("Accl:"));
    Serial.print(F("Accl:"));
    x = a->acceleration.x;
    y = a->acceleration.y;
    z = a->acceleration.z;
  
    myFile.print(F("Orient:"));
    Serial.print(F("Orient:"));

    x = o->orientation.x;
    y = o->orientation.y;
    z = o->orientation.z;
  

    myFile.print(F("\tx= "));
    myFile.print(x);
    myFile.print(F(" |\ty= "));
    myFile.print(y);
    myFile.print(F(" |\tz= "));
    myFile.println(z);

    Serial.print(F("\tx= "));
    Serial.print(x);
    Serial.print(F(" |\ty= "));
    Serial.print(y);
    Serial.print(F(" |\tz= "));
    Serial.println(z);
    // printEvent(&orientationData);
    // printEvent(&accelerometerData);
  
  

  // sensors_event_t orientationData, accelerometerData;
  // bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  // bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);

  // printEvent(&orientationData);
  // printEvent(&accelerometerData);

  Serial.println(F("--"));
  delay(BNO055_SAMPLERATE_DELAY_MS);
  myFile.close();
}

void printEvent(sensors_event_t* event) {
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    Serial.print(F("Accl:"));
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.print(F("Orient:"));
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  }
  else {
    Serial.print("Unk:");
  }

  Serial.print(F("\tx= "));
  Serial.print(x);
  Serial.print(F(" |\ty= "));
  Serial.print(y);
  Serial.print(F(" |\tz= "));
  Serial.println(z);
}


