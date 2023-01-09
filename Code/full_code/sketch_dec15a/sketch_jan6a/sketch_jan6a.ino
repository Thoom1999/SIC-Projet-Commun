#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SPI.h>
#include <SD.h>

// Connections
// ===========
// Connect SCL to analog 5
// Connect SDA to analog 4
// Connect VDD to 3.3-5V DC
// Connect GROUND to common ground
// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// Connect the GPS TX (transmit) pin to Digital 8
// Connect the GPS RX (receive) pin to Digital 7
// Connect the SD MOSI pin to Digital 11
// Connect the SD MISO pin to Digital 12
// Connect the SD CS pin to Digital 4
// Connect the SD SCK pin to Digital 13

// you can change the pin numbers to match your wiring:
SoftwareSerial mySerial(8, 7);
Adafruit_GPS GPS(&mySerial);

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

File myFile;

void setup() {
    // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  delay(2000);
  Serial.println(F("Adafruit GPS library basic parsing test!"));

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);

  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);

  Serial.println(F("Orientation Sensor Test")); Serial.println(F(""));

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    // Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);

  /* Initialise the SD card */
    Serial.print(F("Initializing SD card..."));

  if (!SD.begin(4)) {
    Serial.println(F("initialization failed!"));
    while (1);
  }
  Serial.println(F("initialization done."));

  delay(1000);
}

uint32_t timer = millis();

void loop() {
    // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {

    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer

    myFile = SD.open("test.txt", FILE_WRITE);
  
  // partie GPS
    if (myFile) {
      myFile.println(GPS.milliseconds);
      myFile.print(F("Fix: ")); myFile.println((int)GPS.fix);

      Serial.println(GPS.milliseconds);     
      Serial.print(F("Fix: ")); Serial.println((int)GPS.fix);

      if (GPS.fix) {
        myFile.print(F("Location: "));
        myFile.print(GPS.latitude, 4); 

        Serial.print(F("Location: "));
        Serial.print(GPS.latitude, 4);
        // Serial.print(GPS.lat);
        myFile.print(F(", "));
        myFile.print(GPS.longitude, 4); 

        Serial.print(F(", "));
        Serial.print(GPS.longitude, 4); 
        // Serial.println(GPS.lon);

        myFile.print(F("Altitude: ")); myFile.println(GPS.altitude);
        myFile.print(F("Satellites: ")); myFile.println((int)GPS.satellites);

        Serial.print(F("Altitude: ")); Serial.println(GPS.altitude);
        Serial.print(F("Satellites: ")); Serial.println((int)GPS.satellites);

        myFile.println(F("--"));
        Serial.println(F("--"));

      }
    }
    
    // partie acceléro
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
  
  myFile.close();
  }

}

// void printEvent(sensors_event_t* event) {
//   double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
//   if (event->type == SENSOR_TYPE_ACCELEROMETER) {
//     myFile.print(F("Accl:"));
//     Serial.print(F("Accl:"));
//     x = event->acceleration.x;
//     y = event->acceleration.y;
//     z = event->acceleration.z;
//   }
//   else if (event->type == SENSOR_TYPE_ORIENTATION) {
//     myFile.print(F("Orient:"));
//     Serial.print(F("Orient:"));

//     x = event->orientation.x;
//     y = event->orientation.y;
//     z = event->orientation.z;
//   }
//   else {
//     myFile.print(F("Unk:"));
//     Serial.print(F("Unk:"));

//   }

//   myFile.print(F("\tx= "));
//   myFile.print(x);
//   myFile.print(F(" |\ty= "));
//   myFile.print(y);
//   myFile.print(F(" |\tz= "));
//   myFile.println(z);

//   Serial.print(F("\tx= "));
//   Serial.print(x);
//   Serial.print(F(" |\ty= "));
//   Serial.print(y);
//   Serial.print(F(" |\tz= "));
//   Serial.println(z);


// }