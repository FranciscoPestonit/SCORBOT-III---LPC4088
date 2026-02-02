/*******************************************************************************
 * \file    menu_I2C.c
 *
 * \brief   Funciones para la interfaz mediante I2C
 */
 
#include "menu_I2C.h"
#include "i2c_lpc40xx.h"
#include "lcd_i2c_lpc40xx.h"
#include "lcd_hd44780_lpc40xx.h"
#include "gpio_lpc40xx.h"
#include "tipos.h"
#include "joystick_analogico.h"
#include "timer_lpc40xx.h"
#include "joystick.h"
#include "SCORBOT_ER_III.h"

uint8_t opcion = 0;


char mensaje_menu[] = "MENU I2C";
char mensaje_menu_2[] = "--> Escoge modo";

char mensaje_modo_1[] = "Demo";
char mensaje_modo_2[] = "Control manual";
char mensaje_modo_3[] = "Programacion PC";


void menu_i2c_inicializar(void){

  
  lcd_inicializar          (16     ,       2,
                            0x27   ,LPC_I2C0,
                            PUERTO5,   PIN2,
                            PUERTO5,   PIN3);

  lcd_encender();
  lcd_encender_retroiluminacion();
  lcd_borrar();
  lcd_cursor_xy(0,0);
  lcd_imprimir_cadena(mensaje_menu);
  lcd_cursor_xy(0,1);
  lcd_imprimir_cadena(mensaje_menu_2);

}

void menu_i2c_seleccionar (void){


while(leer_joystick() != JOYSTICK_CENTRO){

  if(leer_joystick() == JOYSTICK_DERECHA){
    opcion++;
    if(opcion > 3){opcion = 3;}
  }
  if(leer_joystick() == JOYSTICK_IZQUIERDA){
    opcion--;
    if(opcion < 1){opcion = 1;}
  }
  
  switch(opcion){
  
    case 1:
      lcd_borrar();
      lcd_cursor_xy(0,0);
      lcd_imprimir_cadena(mensaje_menu);
      lcd_cursor_xy(0,1);
      lcd_imprimir_cadena(mensaje_modo_1);
      break;
    case 2:
      lcd_borrar();
      lcd_cursor_xy(0,0);
      lcd_imprimir_cadena(mensaje_menu);
      lcd_cursor_xy(0,1);
      lcd_imprimir_cadena(mensaje_modo_2);
      break;
    case 3:
      lcd_borrar();
      lcd_cursor_xy(0,0);
      lcd_imprimir_cadena(mensaje_menu);
      lcd_cursor_xy(0,1);
      lcd_imprimir_cadena(mensaje_modo_3);
      break;
    default:
    break;
    
  }
  timer_retardo_ms(TIMER1, 200);
}

if(opcion == 1){modo_demo();}
if(opcion == 2){}
if(opcion == 3){}


return;  
}