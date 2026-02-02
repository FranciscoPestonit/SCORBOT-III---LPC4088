/*******************************************************************************
 * \file    lcd1602_I2C.c
 *
 * \brief   Funciones de manejo de la pantalla LCD de 16x2 caractéres mediante I2C
 *					utilizando el chip expansor de 8 bits mediante I2C PCF8574 de NXP.
 */

#include <LPC407x_8x_177x_8x.h>
#include "i2c_lpc40xx.h"
#include "timer_lpc40xx.h"
#include "tipos.h"
#include "gpio_lpc40xx.h"
#include "lcd_i2c_lpc40xx.h"

//Variables estáticas globales para hacerlas locales

static uint8_t LCD_ADDR,EN,RW,RS,RETROILUM_PIN,DATOS[4],RETROILUM_EST;
static LPC_I2C_TypeDef *lcd_interfaz_i2c;

/*******************************************************************************
 * \brief       Inicializar las variables de la pantalla LCD y el bus I2C
 *
 * \param[in]   lcd_dir					dirección del chip
 * \param[in]   en							pin del PCF8574 conectado al pin EN de la LCD
 * \param[in]   rw							pin del PCF8574 conectado al pin RW de la LCD
 * \param[in]   rs							pin del PCF8574 conectado al pin RS de la LCD
 * \param[in]   d4							pin del PCF8574 conectado al pin D4 de la LCD
 * \param[in]   d5							pin del PCF8574 conectado al pin D5 de la LCD
 * \param[in]   d6							pin del PCF8574 conectado al pin D6 de la LCD
 * \param[in]   d7							pin del PCF8574 conectado al pin D7 de la LCD
 * \param[in]   retroilum_pin		pin del PCF8574 conectado al pin que habilita la retroiluminación de la LCD
 * \param[in]   retroilum				estado de la retroiluminación
 *
 */
void lcd_i2c_inic (uint8_t lcd_direccion, LPC_I2C_TypeDef *interfaz_i2c,
									 LPC_GPIO_TypeDef *puerto_sda, uint32_t mascara_pin_sda,
									 LPC_GPIO_TypeDef *puerto_scl, uint32_t mascara_pin_scl){
	LCD_ADDR = lcd_direccion;
	lcd_interfaz_i2c = interfaz_i2c;
	EN = 1<<2;
	RW = 1<<1;
	RS = 1<<0;
	DATOS[0] = 1<<4;
	DATOS[1] = 1<<5;
	DATOS[2] = 1<<6;
	DATOS[3] = 1<<7;
	RETROILUM_PIN = 3;
	RETROILUM_EST = ON;
	
	i2c_inicializar(lcd_interfaz_i2c,400000,puerto_sda,mascara_pin_sda,puerto_scl,mascara_pin_scl);

}
/*******************************************************************************
 * \brief       Envia el dato a la LCD
 *
 * \param[in]   dato		dato que se ha de enviar
 * \param[in]   modo		modo del dato (comando, dato o 4bits)
 *
 */
void lcd_i2c_enviar (uint8_t dato,uint8_t modo){
	
	if ( modo == EIGHT_BITS ){
		lcd_i2c_escribir_4bits(dato>>4, COMMAND );
  }else{
		lcd_i2c_escribir_4bits((dato>>4), modo );
		timer_retardo_us(TIMER0,50);
    lcd_i2c_escribir_4bits((dato&0x0F), modo);
		timer_retardo_us(TIMER0,50);
	}
}

/*******************************************************************************
 * \brief       Reordena el dato a enviar para que se ajuste a los pines a los
 *							que están conectados
 *
 * \param[in]   dato		dato que se ha de enviar
 * \param[in]   modo		modo del dato (comando, dato o 4bits)
 *
 */
void lcd_i2c_escribir_4bits (uint8_t dato, uint8_t modo){
	 
	uint8_t pinMapValue = 0;
	uint8_t i;
   
	// Mapeado del dato a enviar a los pines
	for (i=0;i<4;i++){
		if ((dato & 1) == 1){
			 pinMapValue |= DATOS[i];
		}
		dato = (dato>>1);
	}

	// Si el modo es DATA
	if (modo == DATA){
		modo = RS;
	}
	
	pinMapValue |= modo | (RETROILUM_EST<<RETROILUM_PIN);
	lcd_i2c_pulso_enable (pinMapValue);
}

/*******************************************************************************
 * \brief       Envia los datos mediante I2C a la pantalla LCD y el pulso de
 *							Enable que marca la captura de este por el controlador de la LCD
 *
 * \param[in]   dato	dato que se ha de enviar
 *
 */
void lcd_i2c_pulso_enable (uint8_t dato){
	
	i2c_start(I2C0);
	i2c_transmitir_byte(I2C0,LCD_ADDR<<1);
	i2c_transmitir_byte(I2C0,dato | EN);
	i2c_transmitir_byte(I2C0,dato & ~EN);
	i2c_stop(I2C0);
}

/*******************************************************************************
 * \brief       Establece el estado de la retroiluminación de la LCD.
 *
 * \param[in]   estado	Estado de la retroiluminación ON/OFF
 *
 */
void lcd_i2c_retroiluminacion(uint8_t estado){
	
	RETROILUM_EST = estado;
	i2c_start(I2C0);
	i2c_transmitir_byte(I2C0,LCD_ADDR<<1);
	i2c_transmitir_byte(I2C0,RETROILUM_EST<<RETROILUM_PIN);
	i2c_stop(I2C0);
	
}
