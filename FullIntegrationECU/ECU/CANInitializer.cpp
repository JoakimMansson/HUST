#include "CANInitializer.h"

#define can_tx 8
#define can_rx 9

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
