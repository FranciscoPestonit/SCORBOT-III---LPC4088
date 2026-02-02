/*******************************************************************************
 * \file    lcd_hd44780_lpc4078.h
 *
 * \brief   Funciones de manejo de la pantalla LCD de 16x2 caractéres basadas en
 *					en el controlador HD44780 e interfaz bus de datos de 4bits mediante I2C.
 */
 
#include <LPC407x_8x_177x_8x.h>
#include "lcd_hd44780_lpc40xx.h"
#include "lcd_i2c_lpc40xx.h"
#include "timer_lpc40xx.h"
#include "tipos.h"

//Variables de estado de la LCD

char lcd_display = LCD_DISPLAYON;
char lcd_cursor = LCD_CURSOROFF;
char lcd_parpadeo = LCD_BLINKOFF;

static uint8_t _fila_offsets[4], numfilas, numcolumnas;

//Carácteres personalizados para la pantalla LCD

char Grados[8] 		= {0x6,0x9,0x9,0x6,0x0,0x0,0x0,0x0};
char Campana[8]		= {0x4,0xe,0xe,0x1f,0x1f,0x0,0x4,0x0};
char Rayo[8] 			= {0x3,0x6,0xc,0x1c,0xf,0x6,0xc,0x18};
char Bombilla[8]	= {0xe,0x15,0x1b,0x15,0xe,0xa,0xe,0x0};
char Tick[8] 			= {0x0,0x1,0x3,0x16,0x1c,0x8,0x0,0x0};
char Cruz[8] 			= {0x0,0x1b,0xe,0x4,0xe,0x1b,0x0,0x0};

/*******************************************************************************
 * \brief       Envia una instrucción a la pantalla LCD.
 *
 * \param[in]   instruccion		instrucción a enviar
 *
 */
void lcd_escribir_ir(uint8_t instruccion){

    lcd_i2c_enviar(instruccion,COMMAND);
}

/*******************************************************************************
 * \brief       Envia un dato a la pantalla LCD.
 *
 * \param[in]   dato		dato a enviar
 *
 */
void lcd_escribir_dr(uint8_t dato){
	
    lcd_i2c_enviar(dato,DATA);
}

/*******************************************************************************
 * \brief       Inicializa la pantalla LCD en modo de bus de datos de 4bits.
 *							La secuencia de inicialización es la recomendada en la hoja de
 *							características del fabricande del HD44780.
 *
 */
void lcd_inicializar(uint8_t columnas, uint8_t filas,
										 uint8_t lcd_direccion, LPC_I2C_TypeDef *interfaz_i2c,
										 LPC_GPIO_TypeDef *puerto_sda, uint32_t mascara_pin_sda,
										 LPC_GPIO_TypeDef *puerto_scl, uint32_t mascara_pin_scl){	
	numfilas = filas;
	numcolumnas = columnas;
	lcd_offsets_filas(0x00, 0x40, 0x00 + columnas, 0x40 + columnas);
	
	timer_inicializar(TIMER0);
	
	//Inicializamos el bus y configuramos los pines del expansor I2C
	lcd_i2c_inic(0x27,interfaz_i2c,puerto_sda,mascara_pin_sda,puerto_scl,mascara_pin_scl);
	
	//Esperamos más de 15ms tras aplicar la alimentación
	timer_retardo_us(TIMER0,1000000);
	
	//Enviamos la instrucción de configuración para el modo 8bits
	//Esperamos más de 4.1ms
	lcd_i2c_enviar(LCD_FUNCTIONSET | LCD_8BITMODE, EIGHT_BITS);
	timer_retardo_us(TIMER0,4500);
	
	//Enviamos la instrucción de configuración para el modo 8bits
	//Esperamos más de 100us
	lcd_i2c_enviar(LCD_FUNCTIONSET | LCD_8BITMODE, EIGHT_BITS);
	timer_retardo_us(TIMER0,150);
	
	//Enviamos la instrucción de configuración para el modo 8bits
	//Esperamos a que se ejecute la instrucción (más de 37us)
	lcd_i2c_enviar(LCD_FUNCTIONSET | LCD_8BITMODE, EIGHT_BITS);
	timer_retardo_us(TIMER0,50);

	//Enviamos la instrucción de configuración para el modo 4bits
	//Esperamos a que se ejecute la instrucción (más de 37us)
	lcd_i2c_enviar(LCD_FUNCTIONSET | LCD_4BITMODE, EIGHT_BITS);
	timer_retardo_us(TIMER0,50);
	
	//Configuramos el controlador para 2 lines, modo 4 bits y caracteres de 5x8 pixeles
	lcd_escribir_ir(LCD_FUNCTIONSET | LCD_2LINE | LCD_4BITMODE | LCD_5x8DOTS);
	timer_retardo_us(TIMER0,50);
	
	//Borramos la pantalla
	lcd_escribir_ir(LCD_CLEARDISPLAY);
	timer_retardo_us(TIMER0,50);
	
	//Retornamos el registro de dirección a 0 y reseteamos el desplazamiento
	//Esperamos que se complete la instrucción (más de 1.51ms)
	lcd_escribir_ir(LCD_RETURNHOME);
	timer_retardo_us(TIMER0,2000);
	
	//Configuramos el modo de entrada de los datos:
	//Autoincremento de la dirección y no desplazar la pantalla
	lcd_escribir_ir(LCD_ENTRYMODESET | LCD_ENTRYINCREMENTON | LCD_ENTRYSHIFTOFF);
	timer_retardo_us(TIMER0,50);
	
	//Configuramos el control del cursor y la pantalla
	lcd_escribir_ir(LCD_DISPLAYCONTROL | lcd_display | lcd_cursor | lcd_parpadeo);
	timer_retardo_us(TIMER0,50);
	
	//Definimos los caracteres personalizados y los almacenamos en la CGRAM del controlador
	lcd_definir_caracteres(1,Grados);
	lcd_definir_caracteres(2,Campana);
	lcd_definir_caracteres(3,Rayo);
	lcd_definir_caracteres(4,Bombilla);
	lcd_definir_caracteres(5,Tick);
	lcd_definir_caracteres(6,Cruz);
}

/*******************************************************************************
 * \brief       Borra la pantalla LCD completa. Para evitar el retardo que hay
 *							que esperar hasta que la pantalla se borra mediante el comando
 *							Clear y Home se escriben 16 espacios en cada fila.
 *
 */
void lcd_borrar(void){
	
	lcd_imprimir_cadena_xy("                    ", 0, 0);
	lcd_imprimir_cadena_xy("                    ", 0, 1);
	
	if (numcolumnas == 20){
		lcd_imprimir_cadena_xy("                    ", 0, 2);
		lcd_imprimir_cadena_xy("                    ", 0, 3);
	}
	
}

/*******************************************************************************
 * \brief       Posiciona el cursor en la posición indicada.
 *
 * \param[in]   x		Columna
 * \param[in]   y		Fila
 *
 */
void lcd_cursor_xy(int x, int y){

  if ( y >= numfilas ) {
    y = numfilas - 1;    // we count rows starting w/0
  }
	
  lcd_escribir_ir(LCD_SETDDRAMADDR | (x + _fila_offsets[y]));
}

/*******************************************************************************
 * \brief       Imprime un carácter el posición actual del cursor.
 *
 * \param[in]   c		Carácter a imprimir en pantalla
 *
 */
void lcd_imprimir_caracter(char c){
    lcd_escribir_dr(c);
}

/*******************************************************************************
 * \brief       Imprime un carácter en la posición indicada.
 *
 * \param[in]   x		Columna
 * \param[in]   y		Fila
 * \param[in]   c		Carácter a imprimir en pantalla
 *
 */
void lcd_imprimir_caracter_xy(char c, int x, int y){
	lcd_cursor_xy(x, y);
	lcd_escribir_dr(c);
}

/*******************************************************************************
 * \brief       Imprime una cadena de caracteres a partir de la posición actual
 *							del cursor.
 *
 * \param[in]   ptr		Puntero a la cadena de caracteres
 *
 */
void lcd_imprimir_cadena(char *ptr){
    
	while(*ptr){
			lcd_escribir_dr(*ptr++);
	}
}

/*******************************************************************************
 * \brief       Imprime una cadena de caracteres a partir de la posición indicada.
 *
 * \param[in]   ptr		Puntero a la cadena de caracteres
 * \param[in]   x			Columna
 * \param[in]   y			Fila
 *
 */
void lcd_imprimir_cadena_xy(char *ptr, int x, int y){

	lcd_cursor_xy(x, y);

	while(*ptr){
		lcd_escribir_dr(*ptr++);
	}
}

/*******************************************************************************
 * \brief       Guarda en la posición de la CGRAM del controlador de la LCD un 
 *							carácter personalizado
 *
 * \param[in]   posicion		Posición de la CGRAM (máximo 8 caracteres personalizados)
 * \param[in]   caracter[]	Array con la matriz 5x8 de bits. 1 enciende el pixel.
 *
 */
void lcd_definir_caracteres(char posicion, char caracter[]){
   
	int i;
	
	posicion &= 0x7; //La posicion máxima es la 0111

	lcd_i2c_enviar(LCD_SETCGRAMADDR | (posicion << 3),COMMAND);
	timer_retardo_us(TIMER0,30);

	for (i=0; i<8; i++) {
		lcd_escribir_dr(caracter[i]); //Enviamos cada byte del array de píxeles
		timer_retardo_us(TIMER0,40);
	}
}

/*******************************************************************************
 * \brief		Muestra el cursor de la pantalla LCD
 *
 */
void lcd_mostrar_cursor(void){
	
	lcd_cursor = LCD_CURSORON;
	lcd_escribir_ir(LCD_DISPLAYCONTROL | lcd_display | lcd_cursor | lcd_parpadeo);
	
}

/*******************************************************************************
 * \brief		Oculta el cursor de la pantalla LCD
 *
 */
void lcd_ocultar_cursor(void){
	
	lcd_cursor = LCD_CURSOROFF;
	lcd_escribir_ir(LCD_DISPLAYCONTROL | lcd_display | lcd_cursor | lcd_parpadeo);
	
}

/*******************************************************************************
 * \brief		Muestra el parpadeo del cursor de la pantalla LCD
 *
 */
void lcd_mostrar_parpadeo_cursor(void){
	
	lcd_cursor = LCD_BLINKON;
	lcd_escribir_ir(LCD_DISPLAYCONTROL | lcd_display | lcd_cursor | lcd_parpadeo);
	
}

/*******************************************************************************
 * \brief		Oculta el parpadeo del cursor de la pantalla LCD
 *
 */
void lcd_ocultar_parpadeo_cursor(void){
	
	lcd_cursor = LCD_BLINKOFF;
	lcd_escribir_ir(LCD_DISPLAYCONTROL | lcd_display | lcd_cursor | lcd_parpadeo);
	
}

/*******************************************************************************
 * \brief		Enciende la pantalla LCD
 *
 */
void lcd_encender(void){
	
	lcd_cursor = LCD_DISPLAYON;
	lcd_escribir_ir(LCD_DISPLAYCONTROL | lcd_display | lcd_cursor | lcd_parpadeo);
	
}

/*******************************************************************************
 * \brief		Apaga la pantalla LCD
 *
 */
void lcd_apagar (void){
	
	lcd_cursor = LCD_DISPLAYOFF;
	lcd_escribir_ir(LCD_DISPLAYCONTROL | lcd_display | lcd_cursor | lcd_parpadeo);
	
}

/*******************************************************************************
 * \brief		Enciende la retroiluminación la pantalla LCD
 *
 */
void lcd_encender_retroiluminacion (void){
	
	lcd_i2c_retroiluminacion(ON);
	
}

/*******************************************************************************
 * \brief		Apaga la retroiluminación pantalla LCD
 *
 */
void lcd_apagar_retroiluminacion (void){
	
	lcd_i2c_retroiluminacion(OFF);
	
}

void lcd_offsets_filas(int fila0, int fila1, int fila2, int fila3)
{
  _fila_offsets[0] = fila0;
  _fila_offsets[1] = fila1;
  _fila_offsets[2] = fila2;
  _fila_offsets[3] = fila3;
}
