/*******************************************************************************
 * \file    lcd1602_I2C.h
 *
 * \brief   Funciones de manejo de la pantalla LCD de 16x2 caractéres mediante I2C
 *					utilizando el chip expansor de 8 bits mediante I2C PCF8574 de NXP.
 */
 
#ifndef lcd_i2c_lpc40xx_H
#define lcd_i2c_lpc40xx_H

#include <LPC407x_8x_177x_8x.h>
#include "tipos.h"

#define COMMAND   0
#define DATA      1
#define EIGHT_BITS	2
#define ON				1
#define OFF				0

//Prototipos de funciones
void lcd_i2c_inic (uint8_t lcd_direccion, LPC_I2C_TypeDef *interfaz_i2c,
									 LPC_GPIO_TypeDef *puerto_sda, uint32_t mascara_pin_sda,
									 LPC_GPIO_TypeDef *puerto_scl, uint32_t mascara_pin_scl);
void lcd_i2c_enviar (uint8_t dato,uint8_t modo);
void lcd_i2c_escribir_4bits (uint8_t dato, uint8_t modo);
void lcd_i2c_pulso_enable (uint8_t dato);
void lcd_i2c_retroiluminacion(uint8_t estado);

#endif /*LCD1602_I2C_H*/
