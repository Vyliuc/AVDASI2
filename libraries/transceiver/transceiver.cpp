#include "transceiver.h"

uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];

void transceiverSetup(RH_RF69 rf69, RHReliableDatagram rf69_manager)
{
  Serial.begin(115200);
  //while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer

  pinMode(LED, OUTPUT);
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  Serial.println("RFM69 Test!");
  Serial.println();

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  if (!rf69_manager.init()) {
      Serial.println("RFM69 radio init failed");
      while (1);
  }
  Serial.println("RFM69 radio init OK!");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
      Serial.println("setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 };
  rf69.setEncryptionKey(key);

  pinMode(LED, OUTPUT);

  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");
}

String transmit(RH_RF69 rf69, RHReliableDatagram rf69_manager, uint8_t RADIO_RX_ADDRESS, String msg) 
{
  // Wait 1s between transmits, could also 'sleep' here!
  delay(10);
  
  const char* radiopacket = msg.c_str();

  Serial.print("Sending "); 
  Serial.print(radiopacket);
  Serial.print(" to "); 
  Serial.println(RADIO_RX_ADDRESS);

  // Send a message to the DESTINATION!
  if (rf69_manager.sendtoWait((uint8_t*)radiopacket, strlen(radiopacket), RADIO_RX_ADDRESS)) 
  {
      Serial.println("Message sent. Waiting for a reply... "); 
      // Now wait for a reply from the server
      uint8_t len = sizeof(buf);
      uint8_t from;

      if (rf69_manager.recvfromAckTimeout(buf, &len, 2000, &from)) {
          buf[len] = 0; // zero out remaining string

          Serial.print("Got reply from #"); Serial.print(from);
          Serial.print(" [RSSI :");
          Serial.print(rf69.lastRssi());
          Serial.print("] : ");
          Serial.println((char*)buf);
          Blink(LED, 40, 3); //blink LED 3 times, 40ms between blinks

          return String((char*)buf);
      }
      else {
          Serial.println("No reply, is anyone listening?");
      }
  }
  else {
      Serial.println("Sending failed (no ack)");
  }

  return "";
}

String receive(RH_RF69 rf69, RHReliableDatagram rf69_manager, int pitchAngle)
{
  if (rf69_manager.available())
  {
    Serial.println("Waiting for the message... ");
    // Wait for a message addressed to us from the client
    uint8_t len = sizeof(buf);
    uint8_t from;
    String responseMsg = "";

    if (rf69_manager.recvfromAck(buf, &len, &from)) {
      buf[len] = 0; // zero out remaining string
      
      Serial.print("Got packet from #"); Serial.print(from);
      Serial.print(" [RSSI :");
      Serial.print(rf69.lastRssi());
      Serial.print("] : ");
      Serial.println((char*)buf);
      Blink(LED, 40, 3); //blink LED 3 times, 40ms between blinks

      responseMsg = getResponseMsg(String((char*)buf), pitchAngle);

      const char* responseMsgChar = responseMsg.c_str();

      // Send a reply back to the originator cliet
      if (!rf69_manager.sendtoWait((uint8_t*)responseMsgChar, strlen(responseMsgChar), from)) 
      {
        Serial.println("Sending failed (no ack)");
      }
      else {
        return responseMsg;
      }
    }
  }

  return "";
}

void Blink(byte PIN, byte DELAY_MS, byte loops) {
  for (byte i = 0; i < loops; i++) {
      digitalWrite(PIN, HIGH);
      delay(DELAY_MS);
      digitalWrite(PIN, LOW);
      delay(DELAY_MS);
  }
}

String getResponseMsg(String msg, int pitchAngle) 
{
  String pitchValString = String(pitchAngle);
  String pitchVal = " Pitch Angle: " + pitchValString;
  
  if (msg == "Manual") return ("Manual Mode Activated!" + pitchVal);
  else if (msg == "Auto") return ("Auto Mode Activated!" + pitchVal);
  else if (msg == "Neutral") return ("Neutral Mode Activated!" + pitchVal);
  else if (msg.indexOf("PotValue:") != -1) return (msg + pitchVal);
  else return "";
}
