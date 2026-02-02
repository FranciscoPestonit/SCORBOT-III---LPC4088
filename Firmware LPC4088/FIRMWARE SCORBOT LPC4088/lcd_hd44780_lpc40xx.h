/*******************************************************************************
 * \file    lcd_hd44780_lpc4078.h
 *
 * \brief   Funciones de manejo de la pantalla LCD de 16x2 caractéres basadas en
 *					en el controlador HD44780 e interfaz de bus de datos de 4bits.
 */
 
#ifndef LCD_HD44780_LPC4078_H
#define LCD_HD44780_LPC4078_H

#include <LPC407x_8x_177x_8x.h>
#include "tipos.h"

//Comandos de Instrucciones de la LCD
#define LCD_CLEARDISPLAY        0x01
#define LCD_RETURNHOME          0x02
#define LCD_ENTRYMODESET        0x04
#define LCD_DISPLAYCONTROL      0x08
#define LCD_CURSORSHIFT         0x10
#define LCD_FUNCTIONSET         0x20
#define LCD_SETCGRAMADDR        0x40
#define LCD_SETDDRAMADDR        0x80

//Modos de Entrada de los datos
#define LCD_ENTRYINCREMENTOFF		0x00
#define LCD_ENTRYINCREMENTON		0x02
#define LCD_ENTRYSHIFTON				0x01
#define LCD_ENTRYSHIFTOFF				0x00

//Modos de control del cursor
#define LCD_DISPLAYON           0x04
#define LCD_DISPLAYOFF          0x00
#define LCD_CURSORON            0x02
#define LCD_CURSOROFF           0x00
#define LCD_BLINKON             0x01
#define LCD_BLINKOFF            0x00

//Modos de control del desplazamiento de la pantalla
#define LCD_DISPLAYMOVE         0x08
#define LCD_CURSORMOVE          0x00
#define LCD_MOVERIGHT           0x04
#define LCD_MOVELEFT            0x00

//Modos de configuracion del controlador de la LCD
#define LCD_8BITMODE            0x10
#define LCD_4BITMODE            0x00
#define LCD_2LINE               0x08
#define LCD_1LINE               0x00
#define LCD_5x10DOTS            0x04
#define LCD_5x8DOTS             0x00


//Prototipos de funciones
void lcd_escribir_ir(uint8_t instruccion);
void lcd_escribir_dr(uint8_t dato);
void lcd_inicializar(uint8_t columnas, uint8_t filas,
										 uint8_t lcd_direccion, LPC_I2C_TypeDef *interfaz_i2c,
										 LPC_GPIO_TypeDef *puerto_sda, uint32_t mascara_pin_sda,
										 LPC_GPIO_TypeDef *puerto_scl, uint32_t mascara_pin_scl);
void lcd_borrar(void);
void lcd_cursor_xy(int x, int y);
void lcd_imprimir_caracter(char c);
void lcd_imprimir_caracter_xy(char c, int x, int y);
void lcd_imprimir_cadena(char *ptr);
void lcd_imprimir_cadena_xy(char *ptr, int x, int y);
void lcd_definir_caracteres(char location, char caracter[]);
void lcd_mostrar_cursor(void);
void lcd_ocultar_cursor(void);
void lcd_mostrar_parpadeo_cursor(void);
void lcd_ocultar_parpadeo_cursor(void);
void lcd_encender(void);
void lcd_apagar (void);
void lcd_encender_retroiluminacion (void);
void lcd_apagar_retroiluminacion (void);
void lcd_offsets_filas(int fila0, int fila1, int fila2, int fila3);

#endif /* LCD_HD44780_LPC4078_H */
