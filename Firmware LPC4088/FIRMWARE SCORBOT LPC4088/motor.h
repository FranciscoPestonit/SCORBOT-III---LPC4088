/*******************************************************************************
 * \file    motor.h
 *
 * \brief   Funciones de manejo de un motor mediante PWM.
 */

#ifndef MOTOR_H
#define MOTOR_H

#include <LPC407x_8x_177x_8x.h>
#include "gpio_lpc40xx.h"
#include "tipos.h"

// FUNCIÓN MAP PARA EL ESCALADO DE UNA VARIABLE ENTRE 2 RANGOS, BASADA EN LA FUNCIÓN MAP DE ARDUINO.
#define map(x,in_min,in_max,out_min,out_max) ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

#define MOTOR_PERIODO_PWM_US      30000                     // Periodo PWM en us
#define MOTOR_T_ON_DETENIDO_US    MOTOR_PERIODO_PWM_US + 1  // Ton para detener el motor en us

//ETIQUETAS PARA LOS PINES DE LOS ENCODERS, DEL PUERTO 0 Y PUERTO 2

#define ENC_A_BASE    PIN13   //PUERTO 2
#define ENC_B_BASE    PIN11   //PUERTO 2
#define ENC_A_HOMBRO  PIN23   //PUERTO 0
#define ENC_B_HOMBRO  PIN9    //PUERTO 0
#define ENC_A_CODO    PIN7    //PUERTO 0
#define ENC_B_CODO    PIN5    //PUERTO 0
#define ENC_A_M4      PIN28   //PUERTO 0
#define ENC_B_M4      PIN27   //PUERTO 0
#define ENC_A_M5      PIN4    //PUERTO 0
#define ENC_B_M5      PIN21   //PUERTO 0
#define ENC_A_PINZA   PIN12   //PUERTO 2
#define ENC_B_PINZA   PIN19   //PUERTO 0

//ETIQUETAS DE LOS PINES DEL PUERTO 0 PARA LOS MICROSWITCHES
#define SW_BASE         PIN22
#define SW_HOMBRO       PIN24
#define SW_CODO         PIN26
#define SW_INCLINACION  PIN20
#define SW_ROTACION     PIN12


//ETIQUETAS PARA LOS ERRORES MÁXIMOS

#define ERROR_PULSOS_MAXIMO 75
#define PORCENTAJE_ERROR 0.02

//

#define EEPROM_PAGE_BASE     PAGE1
#define EEPROM_PAGE_HOMBRO   PAGE2
#define EEPROM_PAGE_CODO     PAGE3
#define EEPROM_PAGE_M4       PAGE4
#define EEPROM_PAGE_M5       PAGE5
#define EEPROM_PAGE_PINZA    PAGE6

// Funciones posibles de los pines.
typedef enum{
  NO_INIT,
  STOP,
  RUN,
  CW_P,
  CCW_N,
  HI_Z
} motor_estado_t;

typedef enum{
  INCLINACION,
  ROTACION 
} diferencial_t;

extern diferencial_t diferencial;

typedef struct{
        LPC_GPIO_TypeDef*    puerto;
        uint32_t             mascara_pin;
  __IO  LPC_PWM_TypeDef*     reg;
  __IO  uint32_t*            match;
        uint32_t             funcion_iocon;
  const  uint8_t              canal;
} PWM_PinConfig_TypeDef;


typedef struct{

  LPC_GPIO_TypeDef*    puerto;
  uint32_t             mascara_pin;

} Microswitch_TypeDef;


typedef struct{

  LPC_GPIO_TypeDef*    puerto_P0;
  LPC_GPIO_TypeDef*    puerto_P1;
  uint32_t             mascara_pin_P0;
  uint32_t             mascara_pin_P1;
//  int                  estado_anterior;
//  int                  estado;
//  int                  transicion;
//  uint16_t             perdida_pulsos;
} Encoder_TypeDef;


typedef struct{

  int16_t motorDirection;   //VARIABLE PARA ALMACENAR EL SENTIDO DE GIRO
  
  int16_t previous_motorPosition;
  int16_t motorPosition;
  float targetPosition;
  int16_t controlSignal;
  
  //PID PARAMETERS
  float proportional; //k_p = 0.5
  float integral; //k_i = 3
  float derivative; //k_d = 1
  float feed_forward; //kff

  //PID TIME AND ERROR VARIABLES
  float previousTime; //for calculating delta t
  float previousError; //for calculating the derivative (edot)
  float errorIntegral; //integral error
  float currentTime; //time in the moment of calculation
  float deltaTime; //time difference
  float errorValue; //error
  float edot; //derivative (de/dt)

}  PID_Parameters_TypeDef;

typedef struct{
  const    PWM_PinConfig_TypeDef      pwm_p;
  const    PWM_PinConfig_TypeDef      pwm_n;
                
  const    uint8_t                    pwm_min;
  const    uint8_t                    pwm_max;
  const    char                       nombre[16];
  
  const    float32_t                  pulsos_por_grado;  

           uint8_t                    pagina_eeprom;
           int16_t                    posicion_abs;
           int16_t                    posicion_uart;
  float                               velocidad_medida;
  
           Encoder_TypeDef            encoder;
  
  const    Microswitch_TypeDef        microswitch;
  
  volatile PID_Parameters_TypeDef     PID;
  
  volatile motor_estado_t             estado;

} Motor_TypeDef;

extern Motor_TypeDef* ptr_motores[];

void pwm_inicializar(void);
void motor_inicializar(Motor_TypeDef* motor);
void encoder_inicializar(Motor_TypeDef* motor);
void microswitch_inicializar(Motor_TypeDef* motor);
void rele_inicializar(void);
void rele_ON(void);
void rele_OFF(void);
void PID_inicializar(Motor_TypeDef* motor);


void SCORBOT_inicializar(void); //ESTA FUNCIÓN HABRÁ QUE TERMINAR PASÁNDOLA A SCORBOT_ER_III.h

void calculatePID(Motor_TypeDef* motor);
void actualizarPID(Motor_TypeDef* motor);
void actualizarPID_dif(Motor_TypeDef* motor);

void motor_ajustar_velocidad(Motor_TypeDef* motor, int8_t porcentaje_velocidad);

void motor_ajustar_posicion_relativa(Motor_TypeDef* motor, int16_t position);
void motor_ajustar_posicion_absoluta(Motor_TypeDef* motor, int16_t posicion_absoluta);


void motor_ajustar_posicion_relativa_diferencial(int16_t position         , diferencial_t diferencial);
void motor_ajustar_posicion_absoluta_diferencial(int16_t posicion_absoluta, diferencial_t diferencial);

int16_t convertir_grados_pulsos(Motor_TypeDef* motor, float32_t grados);

void prueba_PWM_motor(void);
void probar_motor(Motor_TypeDef* motor);

void determinar_estado_encoder (Motor_TypeDef* motor);
void determinar_estado_encoder_hombro (Motor_TypeDef* motor);
//void encoder_estado_inicial (Motor_TypeDef* motor);

void guardar_posicion(Motor_TypeDef* motor);

#endif /* MOTOR_H */
