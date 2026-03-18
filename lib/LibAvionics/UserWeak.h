#ifndef ROCKET_AVIONICS_TEMPLATE_USERWEAK_H
#define ROCKET_AVIONICS_TEMPLATE_USERWEAK_H

#include <./Arduino_Extended.h>

extern void UserSetupGPIO();

extern void UserSetupActuator();

extern void UserSetupCDC();

extern void UserSetupUSART();

extern void UserSetupI2C();

extern void UserSetupSPI();

extern void UserThreads();

#endif  //ROCKET_AVIONICS_TEMPLATE_USERWEAK_H
