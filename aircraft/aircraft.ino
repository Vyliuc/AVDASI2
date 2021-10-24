#include <SD.h>
#include <SPI.h>
#include <transceiver.h>

#define RADIO_TX_ADDRESS     69
#define RADIO_RX_ADDRESS     420

File logsFile;

void setup() {
  transceiver_setup(RADIO_TX_ADDRESS);
  
  Serial.begin(9600);
  Serial.print("Initializing SD card...");

  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
    //while (1);
  }
  Serial.println("initialization done.");
}

void loop() 
{
  // constantly listen to the transceiver & check if any data has been received
  String response = receive();

  if (response == "Manual") 
  {

  }
  else if (response == "Auto") 
  {

  }
  else if (response == "Neutral") 
  {

  }
}

void log(String msg) 
{
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  logsFile = SD.open("logs.txt", FILE_WRITE);
  
  // if the file opened okay, write to it:
  if (logsFile) {
    Serial.print("Writing to logs.txt...");
    logsFile.println(msg);
    // close the file:
    logsFile.close();
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
}
