/*******************************************************************************
 * \file    motor.h
 *
 * \brief   Funciones para controlar 2 joysticks analógicos
 */

#ifndef JOYSTICK_ANALOGICO_H
#define JOYSTICK_ANALOGICO_H

#include "gpio_lpc40xx.h"
#include "iocon_lpc40xx.h"

#define JOY_SCOR_1_X  PIN6
#define JOY_SCOR_1_Y  PIN8
#define JOY_SCOR_2_X  PIN22
#define JOY_SCOR_2_Y  PIN27

#define JOY_SCOR_1_NADA      0
#define JOY_SCOR_1_DCHA      1
#define JOY_SCOR_1_IZQ       2
#define JOY_SCOR_1_ARRIBA    3
#define JOY_SCOR_1_ABAJO     4

#define JOY_SCOR_2_NADA      0
#define JOY_SCOR_2_DCHA      1
#define JOY_SCOR_2_IZQ       2
#define JOY_SCOR_2_ARRIBA    3
#define JOY_SCOR_2_ABAJO     4


void joystick_scorbot_inicializar(void);
uint8_t leer_joystick_1(void);
uint8_t leer_joystick_2(void);


#endif /*JOYSTICK_ANALOGICO_H*/