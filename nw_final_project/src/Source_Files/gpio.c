//***********************************************************************************
// Include files
//***********************************************************************************
#include "gpio.h"

//***********************************************************************************
// defined files
//***********************************************************************************


//***********************************************************************************
// global variables
//***********************************************************************************


//***********************************************************************************
// function prototypes
//***********************************************************************************


//***********************************************************************************
// functions
//***********************************************************************************

/***************************************************************************//**
 * @brief
 * configures the gpio pins
 *
 * @details
 * sets up drivestrength and pin modes for all in use gpios
 * these include the leds and the buttons
 *
 * @note
 * if any other gpio even interrupt is triggered we want to assert false because none of them should even be enabled
 *
 ******************************************************************************/
void gpio_open(void){

  // Set LED ports to be standard output drive with default off (cleared)
  GPIO_DriveStrengthSet(LED0_port, gpioDriveStrengthStrongAlternateStrong);
//  GPIO_DriveStrengthSet(LED0_port, gpioDriveStrengthWeakAlternateWeak);
  GPIO_PinModeSet(LED0_port, LED0_pin, gpioModePushPull, LED0_default);

  GPIO_DriveStrengthSet(LED1_port, gpioDriveStrengthStrongAlternateStrong);
//  GPIO_DriveStrengthSet(LED1_port, gpioDriveStrengthWeakAlternateWeak);
  GPIO_PinModeSet(LED1_port, LED1_pin, gpioModePushPull, LED1_default);

  GPIO_PinModeSet(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN, gpioModeInput, false);
  GPIO_PinModeSet(BSP_GPIO_PB1_PORT, BSP_GPIO_PB1_PIN, gpioModeInput, false);

  //pb0 setup
  GPIO_DriveStrengthSet(BUTTON0_port, gpioDriveStrengthWeakAlternateWeak);  //drive strength dooes not matter
  GPIO_PinModeSet(BUTTON0_port, BUTTON0_pin, gpioModeInput, BUTTON0_default);

  //pb1 setup
  GPIO_DriveStrengthSet(BUTTON1_port, gpioDriveStrengthWeakAlternateWeak);
  GPIO_PinModeSet(BUTTON1_port, BUTTON1_pin, gpioModeInput, BUTTON1_default);

  //slider setup
  GPIO_DriveStrengthSet(CSEN0_port, gpioDriveStrengthWeakAlternateWeak);
  GPIO_PinModeSet(CSEN0_port, CSEN0_pin, gpioModeInput, CSEN0_default);

  GPIO_DriveStrengthSet(CSEN1_port, gpioDriveStrengthWeakAlternateWeak);
  GPIO_PinModeSet(CSEN1_port, CSEN1_pin, gpioModeInput, CSEN1_default);

  GPIO_DriveStrengthSet(CSEN2_port, gpioDriveStrengthWeakAlternateWeak);
  GPIO_PinModeSet(CSEN2_port, CSEN2_pin, gpioModeInput, CSEN2_default);

  GPIO_DriveStrengthSet(CSEN3_port, gpioDriveStrengthWeakAlternateWeak);
  GPIO_PinModeSet(CSEN3_port, CSEN3_pin, gpioModeInput, CSEN3_default);
}
