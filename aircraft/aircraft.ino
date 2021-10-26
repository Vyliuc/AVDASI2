#include <SD.h>
#include <SPI.h>
#include <transceiver.h>

#define RADIO_TX_ADDRESS     69
#define RADIO_RX_ADDRESS     96

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// Class to manage message delivery and receipt
RHReliableDatagram rf69_manager(rf69, RADIO_RX_ADDRESS);

// packet counter, we increment per xmission
int16_t packetnum = 0; 

// Dont put this on the stack:
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];

File logsFile;

void setup() {
  transceiver_setup(rf69, rf69_manager, RADIO_TX_ADDRESS);
  
  Serial.begin(9600);
  Serial.print("Initializing SD card...");

  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("initialization failed!");
    //while (1);
  }
  Serial.println("initialization done.");
}

void loop() 
{
  // constantly listen to the transceiver & check if any data has been received
  String response = receive(rf69, rf69_manager, buf);

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
