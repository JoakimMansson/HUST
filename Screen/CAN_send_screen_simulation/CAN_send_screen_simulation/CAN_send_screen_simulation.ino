#include "Serial_CAN_FD.h"

#define BUTTON_SWITCH_SCREEN_PIN 4
#define ORANGE_BUTTON_7 7
#define YELLOW_BUTTON_6 6
#define BROWN_BUTTON_5 5
//#define GREEN_BUTTON_4 4
#define ORANGE_BUTTON3 3
#define RED_BUTTON_2 2
#define PURPLE_BUTTON_17 17
#define BLINK_RIGHT 9
#define BLINK_LEFT 20

#define __ext 0 //Extended CAN frame
#define __rtr 0 // Data frame
#define __fdf 0 // CAN 2.0 frame
#define uart_can can_serial


int counter_gas = 0;
int counter_break = 0;
int pot_counter = 0;
int driving_mode_counter = 2;
int cruise_control_velocity = 100;

bool in_start_screen = true;
bool data_received_flag = false;
bool update_screen_flag = true;
bool start_screen_flag = true;
bool main_screen_flag = false;
bool high_beam_flag = false;
bool high_voltage_flag = true;
bool battery_error_flag = true;
bool mc_error_flag = true;
bool solar_error_flag = true;
bool driving_mode_error_flag = true;
bool error_flag = true;
bool cruise_control_flag = false;
/*--- Eco 0 Race 1 ---*/
bool eco_or_race_mode_flag = true;
bool right_blinker_flag = false;
bool left_blinker_flag = false;
bool hazard_lights = false;
bool potentiometer_flag = false;

/* ------------------- CAN ------------------- */
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

  uart_init(9600);
  can_speed_20(500000);          // set can bus baudrate to 500k
}

void loop() {
  can_send(0x300, __ext, __rtr, __fdf, )
}
