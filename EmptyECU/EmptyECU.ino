#include <SoftwareSerial.h>
#include "Serial_CAN_FD.h"

/* +++++ CAN VARIABLES +++++ */
#define can_tx  2           // tx of serial can module connect to D2
#define can_rx  3           // rx of serial can module connect to D3

unsigned long __id = 0;
unsigned char __ext = 0; // extended frame or standard frame
unsigned char __rtr = 0; // remote frame or data frame
unsigned char __fdf = 0; // can fd or can 2.0
unsigned char __len = 0; // data length
unsigned char __dta[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // data

SoftwareSerial can_serial(can_tx, can_rx);

#define uart_can can_serial

void uart_init(unsigned long baudrate)
{
    uart_can.begin(baudrate);
}

void uart_write(unsigned char c)
{
    uart_can.write(c);

}

unsigned char uart_read()
{
    return uart_can.read();
}

int uart_available()
{
    return uart_can.available();
}

void setup() {
  Serial.begin(9600);
  /* INITIALIZE CAN*/
  uart_init(9600);
  can_speed_20(500000); // set can bus baudrate to 500k  
  //delay(10000);
}

void loop() {
  unsigned char drive_10[] = {0, 0, 250, 68, 0x8f, 0xc2, 0x75, 0x3d};
  can_send(0x501, 0, 0, 0, 8, drive_10);
  delay(10);
  /*
  unsigned char dta[] = {1, 0, 0, 0, 0, 0, 0, 0};
  can_send(0x010, 0, 0, 0, 8, dta);
  delay(10);
  */
}
