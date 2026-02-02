/*******************************************************************************
 * \file    main.c
 * \brief   Ejemplo de manejo de un motor mediante PWM.
 */
#include <LPC407x_8x_177x_8x.h>
#include <stdio.h>
#include <stdlib.h>
#include "timer_lpc40xx.h"
#include "gpio_lpc40xx.h"
#include "uart_lpc40xx.h"
#include "iocon_lpc40xx.h"
#include "joystick.h"
#include "motor.h"
#include "math.h"
#include "menu_I2C.h"
#include "EEPROM_lpc40xx.h"
#include "SCORBOT_ER_III.h"

int main(void){

eeprom_inicializar();

SCORBOT_inicializar();

uart_inicializar(UART0, UART_BAUDRATE_115200, UART_BITS_DATOS_8, UART_PARIDAD_NINGUNA, UART_BITS_STOP_1, PUERTO0, PIN2, PUERTO0, PIN3, NULL);

SCORBOT_HOME();

while(1){

  esperar_recibir_movimientos();

  mover_base(ptr_motores[0]->posicion_uart);
  mover_hombro(ptr_motores[1]->posicion_uart);
  mover_codo(ptr_motores[2]->posicion_uart);
  inclinar_pinza(ptr_motores[3]->posicion_uart);
  rotar_pinza(ptr_motores[4]->posicion_uart);
  abrir_cerrar_pinza(ptr_motores[5]->posicion_uart);
  devolver_grados_reales();

}

}

void GPIO_IRQHandler(void){
//BASE
    if(LPC_GPIOINT->STATR2 & ENC_A_BASE){
      actualizarPID(ptr_motores[0]);
      LPC_GPIOINT->CLR2 = ENC_A_BASE;
    }
//HOMBRO
    if(LPC_GPIOINT->STATR0 & ENC_A_HOMBRO){
      actualizarPID(ptr_motores[1]);
      LPC_GPIOINT->CLR0 = ENC_A_HOMBRO;
    }
//CODO
    
    if(LPC_GPIOINT->STATR0 & ENC_A_CODO){
      actualizarPID(ptr_motores[2]);
      LPC_GPIOINT->CLR0 = ENC_A_CODO;
    }

//PINZA
    
    if(LPC_GPIOINT->STATR0 & ENC_B_PINZA){
      actualizarPID(ptr_motores[5]);
      LPC_GPIOINT->CLR0 = ENC_B_PINZA;
    }

//DIFERENCIAL PARA INCLINACIÓN
    
  if(LPC_GPIOINT->STATR0 & ENC_A_M4)
    {
     if(gpio_leer_pin(PUERTO0, ENC_B_M4)){
      if(diferencial == INCLINACION){
      ptr_motores[3]->PID.motorPosition--;
      ptr_motores[3]->posicion_abs--;
      }
      else{
      ptr_motores[4]->PID.motorPosition++;
      ptr_motores[4]->posicion_abs++;
      uart_transmitir_int16(UART0, ptr_motores[4]->PID.motorPosition);
      uart_transmitir_int16(UART0, LPC_TIM0->TC/1000);
      }
     }
     else{
     if(diferencial == INCLINACION){
     ptr_motores[3]->PID.motorPosition++;
     ptr_motores[3]->posicion_abs++;
     }
     else{
     ptr_motores[4]->PID.motorPosition--;
     ptr_motores[4]->posicion_abs--;
     uart_transmitir_int16(UART0, ptr_motores[4]->PID.motorPosition);
     uart_transmitir_int16(UART0, LPC_TIM0->TC/1000);
     }
     }
     LPC_GPIOINT->CLR0 = ENC_A_M4;
    }

//DIFERENCIAL PARA ROTACIÓN

  if(LPC_GPIOINT->STATR0 & ENC_A_M5)
{
     if(gpio_leer_pin(PUERTO0, ENC_B_M5)){
      ptr_motores[4]->PID.motorPosition--;
      ptr_motores[4]->posicion_abs--;
     }
     else{
     ptr_motores[4]->PID.motorPosition++;
     ptr_motores[4]->posicion_abs++;
     }
     uart_transmitir_int16(UART0, ptr_motores[4]->PID.motorPosition);
     uart_transmitir_int16(UART0, LPC_TIM0->TC/1000);
      LPC_GPIOINT->CLR0 = ENC_A_M5;
    }

}
