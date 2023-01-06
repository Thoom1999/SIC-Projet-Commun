#include <SPI.h>
#include <SD.h>

File myFile;

void setup() {
  Serial.begin(9600);
  while (!Serial) {}
  
  Serial.print("Initializing SD card...");

  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");

  myFile = SD.open("myTest.txt", FILE_WRITE);


  if (myFile) {
    Serial.print("Writing to test.txt...");
    myFile.println("HELLO WORLD");
    myFile.println("I AM ARDUINO");
    myFile.println("PLEASE SUBSCRIBE CLGPROJECT");
    myFile.close();
    Serial.println("done.");
  } 
  
  else {
    Serial.println("error opening test.txt");
  }

  myFile = SD.open("test.txt");
  
  if (myFile) {
    Serial.println("test.txt:");
    
    while (myFile.available()) {
      Serial.write(myFile.read());
    }

    myFile.close();
  } 
  
  else {
    Serial.println("error opening test.txt");
  }
}

void loop() {

}