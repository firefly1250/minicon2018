#include "undercarriage.h"
#include "ble_uart.h"

Undercarriage undercarriage;
BleUart ble;
ServoClass arm(5,1,15);


void setup() {
  undercarriage.Setup();  
  ble.Setup();
  arm.Init(-90,90);
}

void loop() {
  undercarriage.Loop(received);
  ble.Loop();

  const int8_t upper = received[5] != 2 ? received[5] : -1;
  arm.SetDegree(upper);
  delay(10);
}
