#include <SD.h>
#include <SPI.h>
#include <transceiver.h>

#define RADIO_TX_ADDRESS     69
#define RADIO_RX_ADDRESS     96

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// Class to manage message delivery and receipt
RHReliableDatagram rf69_manager(rf69, RADIO_TX_ADDRESS);

File logsFile;

void setup() {
  initSD();
  transceiverSetup(rf69, rf69_manager);
}

void loop() 
{
  // constantly listen to the transceiver & check if any data has been received

  String response = receive(rf69, rf69_manager);

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

void initSD()
{
  Serial.begin(9600);
  Serial.println("Initializing SD card...");

  if (!SD.begin(BUILTIN_SDCARD)) 
  {
    Serial.println("Initialization failed!");
  }
  else 
  {
    Serial.println("Initialization done");
    logToSD("---------------------------------");
    logToSD("SD card initialized successfully!");
  }
}

void logToSD(String msg) 
{
  logsFile = SD.open("logs.txt", FILE_WRITE);
  
  if (logsFile) 
  {
    logsFile.println(msg);
    logsFile.close();
  } 
  else 
  {
    Serial.println("Error opening logs.txt");
  }
}
