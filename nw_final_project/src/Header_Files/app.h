/***************************************************************************//**
 * @file
 * @brief Top level application functions
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#ifndef APP_H
#define APP_H

#include "cmu.h"
#include "gpio.h"
#include <stdlib.h>
#include "em_cmu.h"
#include "capsense.h"
#include "em_emu.h"
#include "os.h"
#include "fifo.h"

//***********************************************************************************
// defined files
//***********************************************************************************
#define PRESSED true
#define NOT_PRESSED false
//***********************************************************************************
// global variables
//***********************************************************************************
typedef enum{
  CH0,
  CH1,
  CH2,
  CH3,
} channel;

typedef enum{
  LEFT,
  RIGHT,
  NONE,
}slider_side;

extern bool pb0, pb1;
extern slider_side cap_dir;
/***************************************************************************//**
 * Initialize application.
 ******************************************************************************/
void app_peripheral_setup(void);
void app_letimer0_open(void);

void read_button0(void);
void read_button1(void);
void read_capsense(void);
void write_led(void);

void pb0_pressed(void);
void pb1_pressed(void);
void cap_pressed(void);
void app_init(void);
void pb0_init(void);
void pb1_init(void);
void cap_init(void);
void em_init(void);
void lab6_init(void);
void led_init(void);
void  App_TimerCallback (void);

void speed_setpoint_init(void);
void vehicle_direction_init(void);
void vehicle_monitor_init(void);
void lcd_display_init(void);
void  App_TimerCallback (void);
void  lcd_timercallback (void);
void game_param_init(void);
#endif  // APP_H
