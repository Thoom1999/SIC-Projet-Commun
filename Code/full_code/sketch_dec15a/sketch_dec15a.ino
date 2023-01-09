#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SD.h>
#include <SPI.h>


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

// you can change the pin numbers to match your wiring:
SoftwareSerial mySerial(8, 7);
Adafruit_GPS GPS(&mySerial);

#define GPSECHO false

/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 2000;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

void setup() {
    // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(9600);
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
    Serial.print(F("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!"));
    while (1);
  }

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

    Serial.println(GPS.milliseconds);
    Serial.print(F("Fix: ")); Serial.println((int)GPS.fix);
    if (GPS.fix) {
      Serial.print(F("Location: "));
      Serial.print(GPS.latitude, 4); 
      // Serial.print(GPS.lat);
      Serial.print(F(", "));
      Serial.print(GPS.longitude, 4); 
      // Serial.println(GPS.lon);

      Serial.print(F("Altitude: ")); Serial.println(GPS.altitude);
      Serial.print(F("Satellites: ")); Serial.println((int)GPS.satellites);

      Serial.println(F("--"));
    }

    sensors_event_t orientationData, accelerometerData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    printEvent(&orientationData);
    printEvent(&accelerometerData);
  }
  // delay(BNO055_SAMPLERATE_DELAY_MS);
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
    Serial.print(F("Unk:"));
  }

  Serial.print(F("\tx= "));
  Serial.print(x);
  Serial.print(F(" |\ty= "));
  Serial.print(y);
  Serial.print(F(" |\tz= "));
  Serial.println(z);
}