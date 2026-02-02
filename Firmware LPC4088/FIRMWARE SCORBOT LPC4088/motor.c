/*******************************************************************************
 * \file    motor->c
 * \brief   Funciones de manejo de un motor mediante PWM.
 */
#include <LPC407x_8x_177x_8x.h>
#include "timer_lpc40xx.h"
#include "gpio_lpc40xx.h"
#include "iocon_lpc40xx.h"
#include "uart_lpc40xx.h"
#include "joystick.h"
#include "motor.h"
#include "EEPROM_lpc40xx.h"
#include "math.h"
#include "stdlib.h"

#define TEST_PWM

#ifdef TEST_PWM
#define PWM_PIN_CFG 0
#else
#define PWM_PIN_CFG IOCON_OD
#endif

//Declaración e inicialización de estructuras tipo Motor_TypeDef

Motor_TypeDef motor1 = {.pwm_p = {.puerto = PUERTO2,
                                  .mascara_pin = PIN4,
                                  .reg = LPC_PWM1,
                                  .match = &LPC_PWM1->MR5,
                                  .funcion_iocon = PWM1_5,
                                  .canal = 5},
                        .pwm_n = {.puerto = PUERTO1,
                                  .mascara_pin = PIN6,
                                  .reg = LPC_PWM0,
                                  .match = &LPC_PWM0->MR4,
                                  .funcion_iocon = PWM0_4,
                                  .canal = 4},
                        .encoder ={.puerto_P0 = PUERTO2,
                                   .puerto_P1 = PUERTO2,
                                   .mascara_pin_P0 = ENC_A_BASE,
                                   .mascara_pin_P1 = ENC_B_BASE},
                        .microswitch = {.puerto = PUERTO0,
                                        .mascara_pin = SW_BASE},
                        .pulsos_por_grado = 3300/310,
                        .estado = NO_INIT,
                        .pwm_min = 20u,
                        .pwm_max = 100u,
                        .pagina_eeprom = EEPROM_PAGE_BASE,
                        .nombre = "Base"};

Motor_TypeDef motor2 = {.pwm_p = {.puerto = PUERTO1,
                                  .mascara_pin = PIN3,
                                  .reg = LPC_PWM0,
                                  .match = &LPC_PWM0->MR2,
                                  .funcion_iocon = PWM0_2,
                                  .canal = 2},
                        .pwm_n = {.puerto = PUERTO1,
                                  .mascara_pin = PIN23,
                                  .reg = LPC_PWM1,
                                  .match = &LPC_PWM1->MR4,
                                  .funcion_iocon = PWM1_4,
                                  .canal = 4},
                        .encoder ={.puerto_P0 = PUERTO0,
                                   .puerto_P1 = PUERTO0,
                                   .mascara_pin_P0 = ENC_A_HOMBRO,
                                   .mascara_pin_P1 = ENC_B_HOMBRO},
                        .microswitch = {.puerto = PUERTO0,
                                        .mascara_pin = SW_HOMBRO},
                        .pulsos_por_grado = 8.72,
                        .estado = NO_INIT,
                        .pwm_min = 20u,
                        .pwm_max = 50u,
                        .pagina_eeprom = EEPROM_PAGE_HOMBRO,
                        .nombre = "Shoulder"};

Motor_TypeDef motor3 = {.pwm_p = {.puerto = PUERTO1,
                                  .mascara_pin = PIN11,
                                  .reg = LPC_PWM0,
                                  .match = &LPC_PWM0->MR6,
                                  .funcion_iocon = PWM0_6,
                                  .canal = 6},
                        .pwm_n = {.puerto = PUERTO1,
                                  .mascara_pin = PIN21,
                                  .reg = LPC_PWM1,
                                  .match = &LPC_PWM1->MR3,
                                  .funcion_iocon = PWM1_3,
                                  .canal = 3},
                        .encoder ={.puerto_P0 = PUERTO0,
                                   .puerto_P1 = PUERTO0,
                                   .mascara_pin_P0 = ENC_A_CODO,
                                   .mascara_pin_P1 = ENC_B_CODO},
                        .microswitch = {.puerto = PUERTO0,
                                        .mascara_pin = SW_CODO},
                        .pulsos_por_grado = 7,
                        .estado = NO_INIT,
                        .pwm_min = 20u,
                        .pwm_max = 100u,
                        .pagina_eeprom = EEPROM_PAGE_CODO,
                        .nombre = "Elbow"};

Motor_TypeDef motor4 = {.pwm_n = {.puerto = PUERTO1,
                                  .mascara_pin = PIN2,
                                  .reg = LPC_PWM0,
                                  .match = &LPC_PWM0->MR1,
                                  .funcion_iocon = PWM0_1,
                                  .canal = 1},
                        .pwm_p = {.puerto = PUERTO1,
                                  .mascara_pin = PIN5,
                                  .reg = LPC_PWM0,
                                  .match = &LPC_PWM0->MR3,
                                  .funcion_iocon = PWM0_3,
                                  .canal = 3},
                                  .estado = NO_INIT,
                        .encoder ={.puerto_P0 = PUERTO0,
                                   .puerto_P1 = PUERTO0,
                                   .mascara_pin_P0 = ENC_A_M4,
                                   .mascara_pin_P1 = ENC_B_M4},
                        .microswitch = {.puerto = PUERTO0,
                                        .mascara_pin = SW_INCLINACION},
                        .pulsos_por_grado = 760/360,
                        .pwm_min = 30u,
                        .pwm_max = 100u,
                        .pagina_eeprom = EEPROM_PAGE_M4,
                        .nombre = "Motor 4"};

Motor_TypeDef motor5 = {.pwm_p = {.puerto = PUERTO1,
                                  .mascara_pin = PIN18,
                                  .reg = LPC_PWM1,
                                  .match = &LPC_PWM1->MR1,
                                  .funcion_iocon = PWM1_1,
                                  .canal = 1},
                        .pwm_n = {.puerto = PUERTO1,
                                  .mascara_pin = PIN20,
                                  .reg = LPC_PWM1,
                                  .match = &LPC_PWM1->MR2,
                                  .funcion_iocon = PWM1_2,
                                  .canal = 2},
                        .encoder ={.puerto_P0 = PUERTO0,
                                   .puerto_P1 = PUERTO0,
                                   .mascara_pin_P0 = ENC_A_M5,
                                   .mascara_pin_P1 = ENC_B_M5},                        
                        .microswitch = {.puerto = PUERTO0,
                                        .mascara_pin = SW_ROTACION},
                        .pulsos_por_grado = 760/360,
                        .estado = NO_INIT,
                        .pwm_min = 30u,
                        .pwm_max = 100u,
                        .pagina_eeprom = EEPROM_PAGE_M5,
                        .nombre = "Motor 5"};

Motor_TypeDef motor6 = {.pwm_n = {.puerto = PUERTO1,
                                  .mascara_pin = PIN7,
                                  .reg = LPC_PWM0,
                                  .match = &LPC_PWM0->MR5,
                                  .funcion_iocon = PWM0_5,
                                  .canal = 5},
                        .pwm_p = {.puerto = PUERTO1,
                                  .mascara_pin = PIN26,
                                  .reg = LPC_PWM1,
                                  .match = &LPC_PWM1->MR6,
                                  .funcion_iocon = PWM1_6,
                                  .canal = 6},
                        .encoder ={.puerto_P0 = PUERTO2,
                                   .puerto_P1 = PUERTO0,
                                   .mascara_pin_P0 = ENC_A_PINZA,
                                   .mascara_pin_P1 = ENC_B_PINZA},                        
                        .pulsos_por_grado = 1,            //LA PINZA SE MIDE EN mm, por lo que se escribe 1 para que no tenga efecto en la multiplicación
                        .estado = NO_INIT,
                        .pwm_min = 20u,
                        .pwm_max = 100u,
                        .pagina_eeprom = EEPROM_PAGE_PINZA,
                        .nombre = "Grip"};


Motor_TypeDef *ptr_motores[] = {&motor1, &motor2, &motor3, &motor4, &motor5, &motor6};

//Declaración de la variable de tipo diferencial_t

diferencial_t diferencial;

/*******************************************************************************
 * \brief       Procedimiento para inicializar los periféricos PWM0 y PWM1 del LPC4088
 *              
 *
 */ 
void pwm_inicializar(void){
  
  // Aplicar alimentación al módulo PWM0/PWM1 poniendo a 1 el bit 5 y 6 de LPC_SC->PCONP
  LPC_SC->PCONP |= (1u<<5) | (1u<<6);

  // Detener ambos PWM
  LPC_PWM0->TCR = 0;
  LPC_PWM1->TCR = 0;

  LPC_PWM0->PC = 0;
  LPC_PWM0->TC = 0;
  LPC_PWM1->PC = 0;
  LPC_PWM1->TC = 0;
    
  /* Ajusta el registro de preescala, PR, para que, cuando se habilite, el
   * contador TC se incremete cada microsegundo.
   */
  LPC_PWM0->PR = PeripheralClock * 1E-6 - 1;
  LPC_PWM1->PR = PeripheralClock * 1E-6 - 1;

  // If no match occurs (i.e. the match value is greater than the PWM rate),
  // the PWM output remains continuously high
  LPC_PWM0->MR0 = MOTOR_PERIODO_PWM_US;
  LPC_PWM0->MR1 = MOTOR_T_ON_DETENIDO_US;
  LPC_PWM0->MR2 = MOTOR_T_ON_DETENIDO_US;
  LPC_PWM0->MR3 = MOTOR_T_ON_DETENIDO_US;
  LPC_PWM0->MR4 = MOTOR_T_ON_DETENIDO_US;
  LPC_PWM0->MR5 = MOTOR_T_ON_DETENIDO_US;
  LPC_PWM0->MR6 = MOTOR_T_ON_DETENIDO_US;
  LPC_PWM0->MCR = (1u<<1);
  
  LPC_PWM1->MR0 = MOTOR_PERIODO_PWM_US;
  LPC_PWM1->MR1 = MOTOR_T_ON_DETENIDO_US;
  LPC_PWM1->MR2 = MOTOR_T_ON_DETENIDO_US;
  LPC_PWM1->MR3 = MOTOR_T_ON_DETENIDO_US;
  LPC_PWM1->MR4 = MOTOR_T_ON_DETENIDO_US;
  LPC_PWM1->MR5 = MOTOR_T_ON_DETENIDO_US;
  LPC_PWM1->MR6 = MOTOR_T_ON_DETENIDO_US;
  LPC_PWM1->MCR = (1u<<1);
  
  LPC_PWM0->PCR = 0;
  LPC_PWM1->PCR = 0;
  
  // Habilita el conteo y el modo PWM en el registro TCR
  LPC_PWM1->TCR = (1u<<0) | (1u<<3);
  LPC_PWM0->TCR = (1u<<0) | (1u<<3);
}
/*******************************************************************************
 * \brief       Configurar la función PWM en los pines asociados a cada motor, asegurando
 *              que comienzan en un ciclo de trabajo igual a 0
 *
 * \param[in]   motor                    Estructura tipo Motor_TypeDef que contiene
 *                                      todos los parámetros necesarios para su control
 *                                   
 */
void motor_inicializar(Motor_TypeDef *motor){
  
  LPC_PWM0->TCR = 0;
  LPC_PWM1->TCR = 0;

  iocon_configurar_pin(motor->pwm_p.puerto, motor->pwm_p.mascara_pin, motor->pwm_p.funcion_iocon, PWM_PIN_CFG);
  iocon_configurar_pin(motor->pwm_n.puerto, motor->pwm_n.mascara_pin, motor->pwm_n.funcion_iocon, PWM_PIN_CFG);
  
  *motor->pwm_p.match = 0;
  *motor->pwm_n.match = 0;
  motor->estado = STOP;

  motor->pwm_p.reg->PCR |= (1u<<(8+motor->pwm_p.canal));
  motor->pwm_n.reg->PCR |= (1u<<(8+motor->pwm_n.canal));
  
  motor->pwm_p.reg->LER = (1u<<motor->pwm_p.canal);
  motor->pwm_n.reg->LER = (1u<<motor->pwm_n.canal);
  
  // Habilita el conteo y el modo PWM en el registro TCR
  LPC_PWM1->TCR = (1u<<0) | (1u<<3);
  LPC_PWM0->TCR = (1u<<0) | (1u<<3);
}

/*******************************************************************************
 * \brief       Inicializar todos los pines conectados a las salidas del encoder como entradas.
 *              Además, inicializa las interrupciones para dichos pines
 * 
 *                                   
 */
void encoder_inicializar(Motor_TypeDef* motor){
  
  LPC_GPIOINT->ENR2 |= ENC_A_BASE;
  LPC_GPIOINT->ENR0 |= ENC_A_HOMBRO;
  LPC_GPIOINT->ENR0 |= ENC_A_CODO;
  LPC_GPIOINT->ENR0 |= ENC_A_M4;
  LPC_GPIOINT->ENR0 |= ENC_A_M5;
  LPC_GPIOINT->ENR0 |= ENC_B_PINZA;
  
  NVIC_ClearPendingIRQ(GPIO_IRQn);    //Borrar cualquier solicitud de interrupción que pudiera haber
  NVIC_EnableIRQ(GPIO_IRQn);          //Habilitar las interrupciones de GPIO
  NVIC_SetPriority(GPIO_IRQn, 3);      //Establecer prioridad en las interrupciones
  __enable_irq();                      //Habilitar las IRQ mediante el registro PRIMASK

}

/*******************************************************************************
 * \brief       Procedimiento para inicializar como entrada todos los pines
 *              relativos a los microswitches de cada motor
 *              FUNCIÓN INNECESARIA DEBIDO A QUE POR DEFECTO ACTÚAN COMO ENTRADA
 */     
void microswitch_inicializar(Motor_TypeDef* motor){

//  iocon_configurar_pin(motor->microswitch.puerto,motor->microswitch.mascara_pin,GPIO);

}



/*******************************************************************************
 * \brief       Inicializar el pin para el relé. Esto probablemente no se utilice en el resultado final
 *
 *
 */
void rele_inicializar(void){
  
gpio_ajustar_dir(PUERTO2, PIN15, DIR_SALIDA);
gpio_pin_a_0(PUERTO2, PIN15);

}
/*******************************************************************************
 * \brief       CERRAR EL RELÉ, QUE ESTÁ NORMALMENTE ABIERTO
 *
 *
 */
void rele_ON (void){

gpio_pin_a_1(PUERTO2, PIN15);

}
/*******************************************************************************
 * \brief       ABRIR EL RELÉ, QUE ESTÁ NORMALMENTE ABIERTO
 *
 *
 */
void rele_OFF (void){

gpio_pin_a_0(PUERTO2, PIN15);

}
/*******************************************************************************
 * \brief       Inicializar todos los campos correspondientes al control PID para cada motor
 *
 * \param[in]   motor                    Estructura tipo Motor_TypeDef que contiene
 *                                      todos los parámetros necesarios para su control
 *
 */
void PID_inicializar(Motor_TypeDef* motor){

  timer_poner_contador_a_0(TIMER0);
  motor->PID.motorDirection = 0;   //VARIABLE PARA ALMACENAR EL SENTIDO DE GIRO
  motor->PID.motorPosition = 0;
  motor->PID.controlSignal = 0;    //VALOR QUE RETORNARÁ LA FUNCIÓN PID_Calculate
  
  //PARÁMETROS PID para respuesta inmediata y frenado rápido
  motor->PID.proportional = 0.5;       //k_p = 0.5
  motor->PID.integral = 0.0001;      //k_i = 0.0001
  motor->PID.derivative = 0.05;        //0.05; //k_d = 0.05

  //VARIABLES DE TIEMPO Y ERRORES
  motor->PID.previousTime = 0;
  motor->PID.previousError = 0;
  motor->PID.errorIntegral = 0;
  motor->PID.currentTime = 0;
  motor->PID.deltaTime = 0;
  motor->PID.errorValue = 0; 
  motor->PID.edot = 0;
  
}
/*******************************************************************************
 * \brief       Procedimiento que ejecuta todas las funciones de inicialización de periféricos
 *
 *                                   
 */
void SCORBOT_inicializar(void){

  uint8_t i = 0;
  // Inicializar la generación PWM y dejar el motor parado
  rele_inicializar(); //EL RELÉ COMIENZA ESTÁNDO ABIERTO AUNQUE ESTÉ NC, YA QUE EL MICRO EN EL RESET ESTÁ ENVIANDO 3.3V
  pwm_inicializar();
  timer_inicializar(TIMER0);
  timer_inicializar(TIMER1);
for(i = 0; i<6 ; i++)
{
  encoder_inicializar(ptr_motores[i]);
  determinar_estado_encoder(ptr_motores[i]);
}

//CUIDADO, AQUÍ NO SE TIENE EN CUENTA EL MOTOR DE LA PINZA YA QUE NO
//HAY UN MICROSWITCH AL QUE CONECTARNOS, POR LO QUE ESTOS CAMPOS NO ESTÁN INICIALIZADOS
//EN "motor6". POR ESO LA VARIABLE DE ITERACIÓN ALCANZA UN VALOR MENOS QUE EN EL RESTO DE BUCLES FOR
for(i = 0; i<5 ; i++)
{
  microswitch_inicializar(ptr_motores[i]);
}

for(i = 0; i<6 ; i++)
{
  motor_inicializar(ptr_motores[i]);
}

for(i = 0; i<6 ; i++)
{
  PID_inicializar(ptr_motores[i]);
  ptr_motores[i]->PID.targetPosition = 0;
}
//EL PID DEL HOMBRO SE ELIGE DISTINTO DEBIDO A SU INERCIA
  // 0.5  1      0.05 dan una respuesta inmediata y frena inmediatamente.
  // 0.01 0.0001 0.05 comienza repentinamente, de repente se frena y sigue despacio hasta terminar progresivamente
  // 0.02 0.0001 0.05 Tiene una respuesta prácticamente uniforme en todo el recorrido
//ptr_motores[1]->PID.proportional = 0.02;
//ptr_motores[1]->PID.derivative = 0.001;
//ptr_motores[1]->PID.integral = 0.05;

  motor_ajustar_velocidad(ptr_motores[0], 0);
  motor_ajustar_velocidad(ptr_motores[1], 0);
  motor_ajustar_velocidad(ptr_motores[2], 0);
  motor_ajustar_velocidad(ptr_motores[3], 0);
  motor_ajustar_velocidad(ptr_motores[4], 0);  
  motor_ajustar_velocidad(ptr_motores[5], 0);    

//Una vez inicializado todo, cerrar el relé

rele_ON();    
}

/*******************************************************************************
 * \brief       Ajustar la velocidad y sentido de giro del motor
 *
 * \param[in]   porcentaje_velocidad    Porcentaje de la velocidad máxima del
 *                                      motor al que se quiere que gire. El
 *                                      valor debe estar entre -100 y 100. Los
 *                                      valores negativos corresponden a sentido
 *                                      de giro antihorario y los valores
 *                                      positivos a sentido de giro horario.     
 */
void motor_ajustar_velocidad(Motor_TypeDef *motor, int8_t porcentaje_velocidad){

  if (porcentaje_velocidad > 100 || porcentaje_velocidad < -100)  return;
  
  // Giro CW con el transistor P
  if (porcentaje_velocidad > 0){
    porcentaje_velocidad = map(porcentaje_velocidad,0,100,motor->pwm_min,motor->pwm_max); 
    *motor->pwm_n.match = 0;
    *motor->pwm_p.match = (MOTOR_PERIODO_PWM_US * porcentaje_velocidad)/100;
    
    motor->estado = RUN;
  
  // Giro CW con el transistor N
  }else if (porcentaje_velocidad < 0){
    porcentaje_velocidad = map(-porcentaje_velocidad,0,100,motor->pwm_min,motor->pwm_max);
    *motor->pwm_p.match = 0;
    *motor->pwm_n.match = (MOTOR_PERIODO_PWM_US * porcentaje_velocidad)/100;
    motor->estado = RUN;

  // Detener
  }else{
    *motor->pwm_p.match = 0;
    *motor->pwm_n.match = 0;
    motor->estado = STOP;
  }
  
//  motor->pwm_p.reg->LER = (1u<<motor->pwm_p.canal);
//  motor->pwm_n.reg->LER = (1u<<motor->pwm_n.canal);

//ACTUALIZACIÓN DE LOS SHADOW REGISTERS LER

if(motor == ptr_motores[0]){
  LPC_PWM0->PCR = (1u<<12);
  LPC_PWM1->PCR = (1u<<13);
  LPC_PWM0->LER = (1u<<4);
  LPC_PWM1->LER = (1u<<5);
}

if(motor == ptr_motores[1]){
  LPC_PWM0->PCR = (1u<<10);
  LPC_PWM1->PCR = (1u<<12);
  LPC_PWM0->LER = (1u<<2);
  LPC_PWM1->LER = (1u<<4);
}

if(motor == ptr_motores[2]){
  LPC_PWM0->PCR = (1u<<14);
  LPC_PWM1->PCR = (1u<<11);
  LPC_PWM0->LER = (1u<<6);
  LPC_PWM1->LER = (1u<<3);
}

if(motor == ptr_motores[3] || motor == ptr_motores[4]){

//LPC_PWM0->LER = 0;
//LPC_PWM1->LER = 0;  

//LPC_PWM0->LER |= (1u<<1);
//LPC_PWM0->LER |= (1u<<3);
//LPC_PWM1->LER |= (1u<<1);
//LPC_PWM1->LER |= (1u<<2);
LPC_PWM0->PCR = (1u<<9) | (1u<<11);
LPC_PWM1->PCR = (1u<<9) | (1u<<10);
LPC_PWM0->LER = 10;
LPC_PWM1->LER = 6;

}


if(motor == ptr_motores[5]){
  LPC_PWM0->LER = (1u<<5);
  LPC_PWM1->LER = (1u<<6);
  LPC_PWM0->PCR = (1u<<13);
  LPC_PWM1->PCR = (1u<<14);
}

  
}

/*******************************************************************************
 * \brief       Calcular en el modelo PID la señal PWM de salida según la cantidad de pulsos de encoder medidos
 *
 * \param[in]   motor                    Estructura tipo Motor_TypeDef que contiene
 *                                      todos los parámetros necesarios para el control PID
 *                                   
 */
void calculatePID(Motor_TypeDef* motor){

  //Utilizar el tiempo de la iteración anterior. Si es la primera iteración, valdrá 0
  motor->PID.previousTime = motor->PID.currentTime;

  //Lectura del tiempo actual en microsegundos
  motor->PID.currentTime = timer_leer(TIMER0);

  //Diferencia de tiempo en segundos
  motor->PID.deltaTime = (motor->PID.currentTime - motor->PID.previousTime) / 1000000;

  //Cálculo de la señal de error e(t)
  motor->PID.errorValue = motor->PID.targetPosition - motor->PID.motorPosition;

  //Cálculo del error derivativo
  motor->PID.edot = (motor->PID.errorValue - motor->PID.previousError) / motor->PID.deltaTime;

  //Cálculo del error integral
  motor->PID.errorIntegral = motor->PID.errorIntegral+(motor->PID.errorValue * motor->PID.deltaTime);

  //Cálculo de la señal de control u(t)
  motor->PID.controlSignal = (motor->PID.proportional * motor->PID.errorValue)
                            +(motor->PID.derivative   * motor->PID.edot)
                            +(motor->PID.integral     * motor->PID.errorIntegral);

  //Guardar el valor del error absoluto para la siguiente iteración
  motor->PID.previousError = motor->PID.errorValue;

}
/*******************************************************************************
 * \brief       Actualiza la señal de control mientras el motor alcanza la posición de consigna
 *
 * \param[in]   motor                   Estructura tipo Motor_TypeDef que contiene
 *                                      todos los parámetros necesarios para el control PID
 */
void actualizarPID(Motor_TypeDef* motor){

  float32_t porcentaje_velocidad = 0;

  determinar_estado_encoder(motor);
  uart_transmitir_int16(UART0, motor->PID.motorPosition);
  uart_transmitir_int16(UART0, LPC_TIM0->TC/1000);
  calculatePID(motor);

  porcentaje_velocidad = motor->PID.controlSignal;          
  if     (porcentaje_velocidad >  100)      {porcentaje_velocidad =  100;}
  else if(porcentaje_velocidad < -100)      {porcentaje_velocidad = -100;}

  if(fabs(motor->PID.errorValue) < 6){
    porcentaje_velocidad = 0;
    motor_ajustar_velocidad(motor, porcentaje_velocidad);
  }
  else{
    motor_ajustar_velocidad(motor, (int8_t) porcentaje_velocidad);
  }
}
/*******************************************************************************
 * \brief       Actualiza la señal de control para los motores del diferencial.
 *              FUNCIÓN INCOMPLETA.
 *
 * \param[in]   motor                   Estructura tipo Motor_TypeDef que contiene
 *                                      todos los parámetros necesarios para el control PID
 */
void actualizarPID_dif(Motor_TypeDef* motor){

 float32_t porcentaje_velocidad = 0;
  
      determinar_estado_encoder(motor);
      uart_transmitir_int16(UART0, motor->PID.motorPosition);
      uart_transmitir_int16(UART0, LPC_TIM0->TC/1000);
      calculatePID(motor);

        porcentaje_velocidad = motor->PID.controlSignal;          
        if     (porcentaje_velocidad >  100)      {porcentaje_velocidad =  100;}
        else if(porcentaje_velocidad < -100)      {porcentaje_velocidad = -100;}

        if(fabs(motor->PID.errorValue) < 6){
        porcentaje_velocidad = 0;
        motor_ajustar_velocidad(motor, porcentaje_velocidad);
        }
        else{
        motor_ajustar_velocidad(motor, (int8_t) porcentaje_velocidad);
        }

}
/*******************************************************************************
 * \brief       Mover un motor a una posición determinada SIN tener en cuenta la posición anterior
 *
 * \param[in]   motor                   Estructura tipo Motor_TypeDef que contiene
 *                                      todos los parámetros necesarios para su control
 *
 * \param[in]   position                Posicion en pulsos que debe seguir el encoder.
 *                                      
 *
 */     
 
void motor_ajustar_posicion_relativa(Motor_TypeDef* motor, int16_t position){

  float32_t porcentaje_velocidad = 0;
  motor->PID.targetPosition = position;
  motor->estado = RUN;
  timer_poner_contador_a_0(TIMER0);
  timer_iniciar_conteo_us(TIMER0);

    
  calculatePID(motor);
  porcentaje_velocidad = motor->PID.controlSignal;
    
  if     (porcentaje_velocidad >  100)      {porcentaje_velocidad =  100;}
  else if(porcentaje_velocidad < -100)      {porcentaje_velocidad = -100;}

  motor_ajustar_velocidad(motor, (int8_t) porcentaje_velocidad);
  
}
/*******************************************************************************
 * \brief       Mover un motor a una posición determinada SIN tener en cuenta la posición anterior
 *
 * \param[in]   motor                    Estructura tipo Motor_TypeDef que contiene
 *                                       todos los parámetros necesarios para su control
 *
 * \param[in]   posicion_absoluta                Posicion referida al HOME
 *                                      
 *
 */   
void motor_ajustar_posicion_absoluta(Motor_TypeDef* motor, int16_t posicion_absoluta){

  int16_t posicion_relativa = 0;
  
  posicion_relativa = posicion_absoluta - motor->posicion_abs;
  
  motor_ajustar_posicion_relativa(motor, posicion_relativa);
  
  guardar_posicion(motor);

  //PENDIENTE AÑADIR AL COMIENZO LÍMITES MÁXIMOS DE LAS POSICIONES PARA IMPEDIR QUE SE ALCANCEN. SI SUCEDE, posicion_relativa = 0 PARA
  //IMPEDIR CUALQUIER MOVIMIENTO CON UN SETPOINT MUY PEQUEÑO. IGUALMENTE, SE DEBERÍA AGREGAR EN MATLAB UNA COMPROBACIÓN ANTES DE ENVIAR NADA
  //O SI NO, HACER LA COMPROBACIÓN EN EL LPC AL RECIBIR LOS MOVIMIENTOS POR LA UART.
/*if(motor->posicion_abs + posicion_absoluta > motor->posicion_maxima){posicion_relativa = 0;}*/
}


/*******************************************************************************
 * \brief       Mover un motor a una posición determinada SIN tener en cuenta la posición anterior
 *              Los dos motores en el mismo sentido hacen la rotación. En sentidos opuestos
 *              hacen el alabeo
 *
 *
 * \param[in]   position                Posicion en pulsos que debe seguir el encoder.
 *
 * \param[in]   diferencial             Variable de estado para determinar el tipo de movimiento del diferencial
 *
 */   

void motor_ajustar_posicion_relativa_diferencial(int16_t position, diferencial_t diferencial){

  float32_t porcentaje_velocidad_3 = 0;
  float32_t porcentaje_velocidad_4 = 0;
  
  if(diferencial == INCLINACION){
  ptr_motores[3]->PID.targetPosition = position; 
  }
  else if(diferencial == ROTACION){
  ptr_motores[4]->PID.targetPosition = position;
  }
  timer_poner_contador_a_0(TIMER0);
  timer_iniciar_conteo_us(TIMER0);

  if(diferencial == INCLINACION){
    do{
          calculatePID(ptr_motores[3]);
          porcentaje_velocidad_3 =  -ptr_motores[3]->PID.controlSignal;
          porcentaje_velocidad_4 =  -ptr_motores[3]->PID.controlSignal; 

      if(porcentaje_velocidad_3 > 100)       {porcentaje_velocidad_3 = 100;}
      else if(porcentaje_velocidad_3 < -100) {porcentaje_velocidad_3 = -100;}
      if(porcentaje_velocidad_4 > 100)       {porcentaje_velocidad_4 = 100;}
      else if(porcentaje_velocidad_4 < -100) {porcentaje_velocidad_4 = -100;}
      motor_ajustar_velocidad(ptr_motores[3], ((int8_t) porcentaje_velocidad_3));
      motor_ajustar_velocidad(ptr_motores[4], ((int8_t) porcentaje_velocidad_4));
      
    }while(fabs(ptr_motores[3]->PID.errorValue) > 2 );

    ptr_motores[3]->PID.motorPosition -= ptr_motores[3]->PID.targetPosition;
  }

  if(diferencial == ROTACION){
    do{
      
      calculatePID(ptr_motores[4]);
      porcentaje_velocidad_3 =  -ptr_motores[4]->PID.controlSignal;
      porcentaje_velocidad_4 =  ptr_motores[4]->PID.controlSignal;

      if(porcentaje_velocidad_3 > 100)       {porcentaje_velocidad_3 = 100;}
      else if(porcentaje_velocidad_3 < -100) {porcentaje_velocidad_3 = -100;}
      if(porcentaje_velocidad_4 > 100)       {porcentaje_velocidad_4 = 100;}
      else if(porcentaje_velocidad_4 < -100) {porcentaje_velocidad_4 = -100;}
      motor_ajustar_velocidad(ptr_motores[3], ((int8_t) porcentaje_velocidad_3));
      motor_ajustar_velocidad(ptr_motores[4], ((int8_t) porcentaje_velocidad_4));
      
    }while(fabs(ptr_motores[4]->PID.errorValue) > 2);
    ptr_motores[4]->PID.motorPosition -= ptr_motores[4]->PID.targetPosition;
  }

    ptr_motores[3]->estado = STOP;
    ptr_motores[4]->estado = STOP;
    
    PID_inicializar(ptr_motores[3]);
    PID_inicializar(ptr_motores[4]);

    motor_ajustar_velocidad(ptr_motores[3], 0);
    motor_ajustar_velocidad(ptr_motores[4], 0);

}
/*******************************************************************************
 * \brief       Mover un motor a una posición determinada SIN tener en cuenta la posición anterior
 *              Los dos motores en el mismo sentido hacen la rotación. En sentidos opuestos
 *              hacen el alabeo
 *
 *
 * \param[in]   posicion_absoluta       Posicion absoluta en pulsos
 *
 * \param[in]   diferencial             Variable de estado para determinar el tipo de movimiento del diferencial
 *
 */   
void motor_ajustar_posicion_absoluta_diferencial(int16_t posicion_absoluta, diferencial_t diferencial){

  int16_t posicion_relativa = 0;

  posicion_relativa = posicion_absoluta - ptr_motores[4]->posicion_abs;
  
  motor_ajustar_posicion_relativa_diferencial(posicion_relativa, diferencial);
  
  //PENDIENTE AÑADIR LÍMITES MÁXIMOS DE LAS POSICIONES PARA IMPEDIR QUE SE ALCANCEN

}
/*******************************************************************************
 * \brief       Procedimiento para convertir los grados en cantidad de pulsos de encoder.
 *              El valor dependerá del motor utilizado y de su correspondiente relación de transmisión.
                PENDIENTE COMPROBAR QUE LAS CONVERSIONES SON CORRECTAS
 *
 * \param[in]   motor                   Estructura tipo Motor_TypeDef que contiene
 *                                      todos los par᭥tros necesarios para su control
 *
 * \param[in]   grados                  Cantidad angular en grados que se desea mover el eslabón.
 *                                      Puede ser positivo o negativo, para indicar un sentido implícito
 *
 */


int16_t convertir_grados_pulsos(Motor_TypeDef* motor, float32_t grados){

int16_t pulsos = 0;

pulsos = grados*motor->pulsos_por_grado;
  
return pulsos;

}
/*******************************************************************************
 * \brief       Procedimiento para comprobar el valor PWM mínimo que acciona cada motor.
 *              
 *              Usar el joystick arriba/abajo para aumentar o disminuir el ciclo de trabajo PWM
 *              
 *              Usar el joystick derecha/izquierda para seleccionar el motor
 *
 *              Pulsar el joystick para iniciar y detener movimiento               
 *
 *              Este procedimiento también es útil para hacer una calibración de los pulsos/grado
 *              NO CONSEGUÍ VISUALIZAR LAS VARIABLES LOCALES EN EL WATCH DEL DEBUGGER
 */
void prueba_PWM_motor(void){

int16_t porcentaje_velocidad = 0;
int8_t num_motor = 0;

while(1){
    
  while(leer_joystick() != JOYSTICK_CENTRO)
  {
    if(leer_joystick() == JOYSTICK_DERECHA) {num_motor++;}
    if(leer_joystick() == JOYSTICK_IZQUIERDA)   {num_motor--;}
    
    if(num_motor > 5){num_motor = 5;}
    if(num_motor < 0){num_motor = 0;}

    timer_retardo_ms(TIMER0,200);
    
  }
  while(leer_joystick() != JOYSTICK_CENTRO)
  {
    if(leer_joystick() == JOYSTICK_ARRIBA) {porcentaje_velocidad++;}
    if(leer_joystick() == JOYSTICK_ABAJO)  {porcentaje_velocidad--;}  
    
    if(porcentaje_velocidad > 100) {porcentaje_velocidad = 100;}
    if(porcentaje_velocidad < -100){porcentaje_velocidad = -100;}
    
    motor_ajustar_velocidad(ptr_motores[num_motor], porcentaje_velocidad);
    
    timer_retardo_ms(TIMER0,200);
  }

  motor_ajustar_velocidad(ptr_motores[num_motor], 0);  
  timer_retardo_ms(TIMER0,200);
}

}
/*******************************************************************************
 * \brief       Función para probar un motor individual con el joystick
 *              
 * \param[in]   motor                    Estructura tipo Motor_TypeDef que contiene
 *                                      todos los par᭥tros necesarios para su control
 */
void probar_motor(Motor_TypeDef* motor){

  int16_t porcentaje_velocidad = 0;

    while(leer_joystick() != JOYSTICK_CENTRO)
  {
    if(leer_joystick() == JOYSTICK_ARRIBA) {porcentaje_velocidad++;}
    if(leer_joystick() == JOYSTICK_ABAJO)  {porcentaje_velocidad--;}  
    
    if(porcentaje_velocidad > 100) {porcentaje_velocidad = 100;}
    if(porcentaje_velocidad < -100){porcentaje_velocidad = -100;}
    
    motor_ajustar_velocidad(motor, porcentaje_velocidad);
    
    timer_retardo_ms(TIMER0,100);
  }

  motor_ajustar_velocidad(motor, 0);
  timer_retardo_ms(TIMER0,500);
  return;

}
/*******************************************************************************
 * \brief       Función que se llama desde el manejador de interrupciones para conocer el estado de un encoder
 *              Esta función también sirve para determinar el estado.
 *              
 * \param[in]   motor                    Estructura tipo Motor_TypeDef que contiene
 *                                      todos los par᭥tros necesarios para su control
 *
 */
void determinar_estado_encoder(Motor_TypeDef* motor){

  int A = gpio_leer_pin(motor->encoder.puerto_P0, motor->encoder.mascara_pin_P0);
  int B = gpio_leer_pin(motor->encoder.puerto_P1, motor->encoder.mascara_pin_P1);

  if(A == B)
  {
     motor->PID.motorDirection = 1;
     motor->PID.motorPosition++;
     motor->posicion_abs++;
  }
  else{
     motor->PID.motorDirection = -1;
     motor->PID.motorPosition--;
     motor->posicion_abs--;
  }
}

/*******************************************************************************
 * \brief       Almacena la posición absoluta para conservar
 *              la referencia en los siguientes movimientos
 *
 *
 */
void guardar_posicion(Motor_TypeDef* motor){

  motor->posicion_abs += motor->PID.motorPosition;
  eeprom_escribir_16bit(motor->pagina_eeprom, motor->posicion_abs);

}