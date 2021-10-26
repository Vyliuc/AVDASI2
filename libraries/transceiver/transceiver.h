#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>
/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 434

#define RFM69_CS      10
#define RFM69_INT     4
#define RFM69_RST     9
#define LED           13

void transceiver_setup(RH_RF69 rf69, RHReliableDatagram rf69_manager, int RADIO_TX_ADDRESS);

String transmit(RH_RF69 rf69, RHReliableDatagram rf69_manager, uint8_t* buf, int RADIO_RX_ADDRESS, String msg);

String receive(RH_RF69 rf69, RHReliableDatagram rf69_manager, uint8_t* buf);

void Blink(byte PIN, byte DELAY_MS, byte loops);

String getResponseMsg(String msg);
