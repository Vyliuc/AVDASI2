#include <tranceiver.h>

void setup() {
  tranceiver_setup();
}

void loop() {
  transmit(data);

  receive(data);
}
