/*******************************************************************************
 * \file    motor->c
 * \brief   Funciones de manejo de un motor mediante PWM.
 */
 
 
 #include "joystick_analogico.h"
 #include "gpio_lpc40xx.h"
 #include "iocon_lpc40xx.h"
 
 void joystick_scorbot_inicializar(void){
 
iocon_configurar_pin(PUERTO2,  JOY_SCOR_1_X, GPIO, IOCON_NO_PULL_UP_NO_PULL_DOWN);
iocon_configurar_pin(PUERTO2,  JOY_SCOR_1_Y, GPIO, IOCON_NO_PULL_UP_NO_PULL_DOWN); 
iocon_configurar_pin(PUERTO1,  JOY_SCOR_2_X, GPIO, IOCON_NO_PULL_UP_NO_PULL_DOWN);
iocon_configurar_pin(PUERTO1,  JOY_SCOR_2_Y, GPIO, IOCON_NO_PULL_UP_NO_PULL_DOWN); 
 
gpio_ajustar_dir(PUERTO2, JOY_SCOR_1_X, DIR_ENTRADA);
gpio_ajustar_dir(PUERTO2, JOY_SCOR_1_Y, DIR_ENTRADA);
gpio_ajustar_dir(PUERTO1, JOY_SCOR_2_X, DIR_ENTRADA);
gpio_ajustar_dir(PUERTO1, JOY_SCOR_2_Y, DIR_ENTRADA);

}

uint8_t leer_joystick_1(void){

uint8_t estado_joystick = 0;
  
  if(gpio_leer_pin(PUERTO2, JOY_SCOR_1_X)){
    estado_joystick = JOY_SCOR_1_DCHA;
  }
  else{
    estado_joystick = JOY_SCOR_1_IZQ;
  }
  if(gpio_leer_pin(PUERTO2, JOY_SCOR_1_Y)){
      estado_joystick = JOY_SCOR_1_ARRIBA;
  }
  else{
    estado_joystick = JOY_SCOR_1_ABAJO;
  }
  
  

return estado_joystick;
}


uint8_t leer_joystick_2(void){

  uint8_t estado_joystick = 0;
  
  if(gpio_leer_pin(PUERTO1, JOY_SCOR_1_X)){
    estado_joystick = JOY_SCOR_1_DCHA;
  }
  else{
    estado_joystick = JOY_SCOR_1_IZQ;
  }
  if(gpio_leer_pin(PUERTO1, JOY_SCOR_1_Y)){
      estado_joystick = JOY_SCOR_1_ARRIBA;
  }
  else{
    estado_joystick = JOY_SCOR_1_ABAJO;
  }
  
  return estado_joystick;
}