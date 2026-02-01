/*******************************************************************************
 * \file    motor.h
 *
 * \brief   Funciones de manejo de un motor mediante PWM.
 */

#include "SCORBOT_ER_III.h"
#include "motor.h"
#include "timer_lpc40xx.h"
#include "EEPROM_lpc40xx.h"
#include "uart_lpc40xx.h"

void modo_demo(void){

SCORBOT_HOME();
mover_base(90);
mover_hombro(40);
mover_codo(25);
inclinar_pinza(30);
rotar_pinza(15);
abrir_cerrar_pinza(-90);

mover_base(10);
mover_hombro(10);
mover_codo(10);
inclinar_pinza(0);
rotar_pinza(0);
abrir_cerrar_pinza(-90);
  
return;
}

void programacion_manual(void){

//Esperar movimientos a ejecutar desde MATLAB a través de la UART

//Inicio Bucle
//Ejecutar movimiento
//Seleccionar primero el número de movimiento a ejecutar (1=BASE, 2=HOMBRO, 3=CODO, 4=INCLINACIÓN, 5=ROTACIÓN, 6=PINZA)
//Seleccionar segundo la magnitud del movimiento(en grados)
//Los datos vendrán en dos arrays


//Convertir posición a grados


//Enviar grados reales a MATLAB a través de la UART



//Volver a Inicio Bucle hasta que se acaben los movimientos




}

/*******************************************************************************
 * \brief       Función para llevar la base a su posición de HOME
 *
 */  
void base_HOME(void){
  
  int8_t porcentaje_velocidad = 0;
  if(eeprom_leer_16bit(EEPROM_PAGE_BASE)>0){porcentaje_velocidad = 20;}
  else {porcentaje_velocidad = -20;}
  
  while(gpio_leer_pin(PUERTO0, SW_BASE)){
    motor_ajustar_velocidad(ptr_motores[0], porcentaje_velocidad);
  }
  ptr_motores[0]->PID.motorPosition = 0;
  motor_ajustar_velocidad(ptr_motores[0], 0);
  
}
/*******************************************************************************
 * \brief       Función para llevar el hombro a su posición de HOME
 *
 */  
void hombro_HOME(void){

  while(gpio_leer_pin(PUERTO0, SW_HOMBRO)){
    motor_ajustar_velocidad(ptr_motores[1], -20);
  }
  ptr_motores[1]->PID.motorPosition = 0;  
  motor_ajustar_velocidad(ptr_motores[1], 0);
  timer_retardo_ms(TIMER0, 1000);
  motor_ajustar_velocidad(ptr_motores[1], 1);
  timer_retardo_ms(TIMER0, 1000);
  ptr_motores[1]->PID.motorPosition = 0;
  motor_ajustar_velocidad(ptr_motores[1], 0);
  timer_retardo_ms(TIMER0, 1000);
  
  while(gpio_leer_pin(PUERTO0, SW_HOMBRO)){
    motor_ajustar_velocidad(ptr_motores[1], -1);
  }
  ptr_motores[1]->PID.motorPosition = 0;
  motor_ajustar_velocidad(ptr_motores[1], 0);

}
/*******************************************************************************
 * \brief       Función para llevar el codo a su posición de HOME
 *
 */  
void codo_HOME(void){
  
  while(gpio_leer_pin(PUERTO0, SW_CODO)){
    motor_ajustar_velocidad(ptr_motores[2], -5);
  }
  ptr_motores[2]->PID.motorPosition = 0;
  motor_ajustar_velocidad(ptr_motores[2], 0);

}

void inclinacion_HOME(void){
  
  while(gpio_leer_pin(PUERTO0, SW_INCLINACION)){
    motor_ajustar_velocidad(ptr_motores[3], -1);
    motor_ajustar_velocidad(ptr_motores[4], -1);
  }
  ptr_motores[3]->PID.motorPosition = 0;
  ptr_motores[4]->PID.motorPosition = 0;
  motor_ajustar_velocidad(ptr_motores[3], 0);
  motor_ajustar_velocidad(ptr_motores[4], 0);
}

void rotacion_HOME(void){
  
//  int8_t velocidad_home_M4;
//  int8_t velocidad_home_M5;
//  if(eeprom_leer_16bit(EEPROM_PAGE_M4)>0){velocidad_home_M4 = -12; velocidad_home_M5 = 10;}
//  else if(eeprom_leer_16bit(EEPROM_PAGE_M4)<0){velocidad_home_M4 = 12; velocidad_home_M5 = -10;}
//  else{
//  velocidad_home_M4 = 0; 
//  velocidad_home_M5 = 0;
//  }

  while(gpio_leer_pin(PUERTO0, SW_ROTACION)){
//    if(velocidad_home_M4 == 0 && velocidad_home_M5 == 0){
//    velocidad_home_M4 = 5; velocidad_home_M5 = 3;
//    }
    motor_ajustar_velocidad(ptr_motores[3], -50);
    motor_ajustar_velocidad(ptr_motores[4], 48);
  }
  
  ptr_motores[3]->PID.motorPosition = 0;
  ptr_motores[4]->PID.motorPosition = 0;
  motor_ajustar_velocidad(ptr_motores[3], 0);
  motor_ajustar_velocidad(ptr_motores[4], 0);
  
}
/*******************************************************************************
 * \brief       Procedimiento para alcanzar la posición HOME mediante los microswitches del brazo.
 *              Este HOME se hará al iniciar una nueva tarea, o cuando lo indique el usuario, en caso
 *              de estar en espera. Se hará a una velocidad reducida. Cada eslabón se moverá hasta que se
 *              active una interrupción por flanco de subida. PENDIENTE HABILITAR DICHA INTERRUPCIÓN
 *
 *              COMENTARIOS IMPORTANTES: TENER CUIDADO AL LLEVAR A HOME EL DIFERENCIAL,
 *              PUES SI HAY DESCOMPENSACIONES PODRÍAN DESACTIVARSE LOS SWITCHES.
 */
void SCORBOT_HOME(void){
  //MOVER CADA MOTOR EN UN SENTIDO PARA LUEGO MOVERLO HACIA EL OTRO
  //HASTA ENCONTRAR UN FLANCO DE SUBIDA PRODUCIDO POR EL MICROSWITCH DEL ESLABÓN
  
  uint8_t i = 0;

  for(i=0; i<6; i++){
    ptr_motores[i]->posicion_abs = eeprom_leer_16bit(ptr_motores[i]->pagina_eeprom);
  }

  //HOMBRO
  hombro_HOME();
  timer_retardo_ms(TIMER0, 1000);   
  ptr_motores[1]->posicion_abs = eeprom_leer_16bit(ptr_motores[1]->pagina_eeprom);

  //CODO
  motor_ajustar_posicion_relativa(ptr_motores[2], 300);
  timer_retardo_ms(TIMER0, 1000);
  ptr_motores[2]->posicion_abs = eeprom_leer_16bit(ptr_motores[2]->pagina_eeprom);
  //ROTACION
  rotacion_HOME();
  timer_retardo_ms(TIMER0, 1000);
  //INCLINACION
  inclinacion_HOME();
  timer_retardo_ms(TIMER0, 1000);
  ptr_motores[3]->posicion_abs = eeprom_leer_16bit(ptr_motores[3]->pagina_eeprom);
  ptr_motores[4]->posicion_abs = eeprom_leer_16bit(ptr_motores[4]->pagina_eeprom);
  //PINZA. Siempre comenzará abierta, para poder determinar un estado inicial
  motor_ajustar_velocidad(ptr_motores[5], -50);
  timer_retardo_ms(TIMER0, 1000);
  ptr_motores[5]->PID.motorPosition = 0;
  motor_ajustar_velocidad(ptr_motores[5], 0);
  timer_retardo_ms(TIMER0, 500);
//  }
  ptr_motores[5]->posicion_abs = eeprom_leer_16bit(ptr_motores[5]->pagina_eeprom);
  //CODO
  codo_HOME();
  timer_retardo_ms(TIMER0, 1000);  
  ptr_motores[2]->posicion_abs = eeprom_leer_16bit(ptr_motores[2]->pagina_eeprom);
  
  //CORRECCION DE INCLINACIÓN DEBIDO AL MOVIMIENTO DEL CODO
  guardar_posicion(ptr_motores[3]);
  guardar_posicion(ptr_motores[3]);
  inclinacion_HOME();
  timer_retardo_ms(TIMER0, 1000);
  ptr_motores[3]->posicion_abs = eeprom_leer_16bit(ptr_motores[3]->pagina_eeprom);
  ptr_motores[4]->posicion_abs = eeprom_leer_16bit(ptr_motores[4]->pagina_eeprom);
  //BASE
    base_HOME();
    ptr_motores[0]->posicion_abs = eeprom_leer_16bit(ptr_motores[0]->pagina_eeprom);
    timer_retardo_ms(TIMER0, 1000);  

  //COLOCAR TODOS LOS SETPOINT A CERO
  for(i=0; i<6; i++){
    ptr_motores[i]->PID.targetPosition = 0;
  }
  //COLOCAR TODAS LAS POSICIONES ABSOLUTAS A CERO
  for(i = 0; i<6; i++){
    ptr_motores[i]->posicion_abs = 0;
    eeprom_escribir_16bit(ptr_motores[i]->pagina_eeprom, ptr_motores[i]->posicion_abs);
  }
}
/*******************************************************************************
 * \brief       Función para mover el motor de la base
 *
 * \param[in]   posicion      Posición a la que queremos mover el eslabón
 *
 */

void mover_base(int16_t posicion){
  
int16_t grados;
grados = convertir_grados_pulsos(ptr_motores[0], posicion);
motor_ajustar_posicion_absoluta(ptr_motores[0], grados);

}
/*******************************************************************************
 * \brief       Función para mover el motor de la hombro
 *
 * \param[in]   posicion      Posición a la que queremos mover el eslabón
 *
 */
void mover_hombro(int16_t posicion){

int16_t grados;
grados = convertir_grados_pulsos(ptr_motores[1], posicion);
motor_ajustar_posicion_absoluta(ptr_motores[1], grados);

}
/*******************************************************************************
 * \brief       Función para mover el motor de la codo
 *
 * \param[in]   posicion      Posición a la que queremos mover el eslabón
 *
 */
void mover_codo(int16_t posicion){

if(posicion == 0){return;}
int16_t grados;
grados = convertir_grados_pulsos(ptr_motores[2], posicion);
motor_ajustar_posicion_absoluta(ptr_motores[2], grados);

}
/*******************************************************************************
 * \brief       Función para inclinar la pinza mediante el diferencial
 *
 * \param[in]   posicion      Posición para saber cuánto queremos inclinar la pinza
 *
 */
void inclinar_pinza(int16_t posicion){

diferencial = INCLINACION;
  
motor_ajustar_posicion_absoluta_diferencial(convertir_grados_pulsos(ptr_motores[3], posicion), diferencial);

}
/*******************************************************************************
 * \brief       Función para rotar la pinza mediante el diferencial
 *
 * \param[in]   posicion      Posición para saber cuánto queremos rotar la pinza
 *
 */
void rotar_pinza(int16_t posicion){

diferencial = ROTACION;
  
motor_ajustar_posicion_absoluta_diferencial(convertir_grados_pulsos(ptr_motores[4], posicion), diferencial);

guardar_posicion(ptr_motores[4]);
}

/*******************************************************************************
 * \brief       Función para abrir y cerrar la pinza
 *
 * \param[in]   posicion      Valor que indicará cuánto queremos abrir o cerrar la pinza
 *
 */
void abrir_cerrar_pinza(int16_t posicion){

int16_t posicion_relativa = posicion;
motor_ajustar_posicion_relativa(ptr_motores[5], posicion_relativa);

}

uint8_t esperar_recibir_movimientos(void){

uint8_t i, j;
uint8_t num_movimiento;
int16_t dato_uart;
uint8_t lowByte;
uint8_t highByte;

  num_movimiento  = uart_esperar_recibir_dato(UART0);
  if(num_movimiento > 20){num_movimiento = 20;}
  
  for(j = 0; j < num_movimiento; j++){
    for(i = 0; i<6; i++){
      lowByte  = uart_esperar_recibir_dato(UART0); // Byte menos significativo
      highByte = uart_esperar_recibir_dato(UART0); // Byte más significativo
      dato_uart = (int16_t)((highByte << 8) | lowByte);
      ptr_motores[i]->posicion_uart[j] = dato_uart;
    }
  }
  
  return num_movimiento;
}

int16_t convertir_pulsos_grados(Motor_TypeDef *motor , int16_t pulsos){

int16_t grados = pulsos*(1/motor->pulsos_por_grado);

return grados;

}

void devolver_grados_reales(void){

  uint8_t i;
  int16_t valor;
  uint8_t lowByte, highByte;
  
  uart_transmitir_dato(UART2, 0xAA);
  
  for (i = 0; i < 6; i++) {
    valor = convertir_pulsos_grados(ptr_motores[i], ptr_motores[i]->posicion_abs);
    lowByte  = (uint8_t)(valor & 0xFF);
    highByte = (uint8_t)((valor >> 8) & 0xFF);

    uart_transmitir_dato(UART2, lowByte);
    uart_transmitir_dato(UART2, highByte);
  }

}

void abrir_pinza(void){

  motor_ajustar_velocidad(ptr_motores[5], 50);
  timer_retardo_ms(TIMER0, 1000);
  ptr_motores[5]->PID.motorPosition = 0;
  motor_ajustar_velocidad(ptr_motores[5], 0);
  timer_retardo_ms(TIMER0, 500);

}


void cerrar_pinza(void){

  motor_ajustar_velocidad(ptr_motores[5], -50);
  timer_retardo_ms(TIMER0, 1000);
  ptr_motores[5]->PID.motorPosition = 0;
  motor_ajustar_velocidad(ptr_motores[5], 0);
  timer_retardo_ms(TIMER0, 500);

}