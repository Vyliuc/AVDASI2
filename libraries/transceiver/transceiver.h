#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>
/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 434

void transceiverSetup(RH_RF69 rf69, RHReliableDatagram rf69_manager, int RFM69_CS, int RFM69_INT, int RFM69_RST, int LED);

String transmit(RH_RF69 rf69, RHReliableDatagram rf69_manager, uint8_t RADIO_RX_ADDRESS, String msg, int LED);

String receive(RH_RF69 rf69, RHReliableDatagram rf69_manager, int pitchAngle, int LED);

void Blink(byte PIN, byte DELAY_MS, byte loops);

String getResponseMsg(String msg, int pitchAngle);
