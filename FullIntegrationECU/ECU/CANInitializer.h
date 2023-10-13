#pragma once
#include <SoftwareSerial.h>
#include "Serial_CAN_FD.h"


void uart_init(unsigned long baudrate);
void uart_write(unsigned char c);
unsigned char uart_read();
int uart_available();
