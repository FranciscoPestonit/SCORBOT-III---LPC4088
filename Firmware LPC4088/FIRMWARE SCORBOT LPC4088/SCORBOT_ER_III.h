/*******************************************************************************
 * \file    SCORBOT_ER_III_H.h
 *
 * \brief   Modos de funcionamiento del SCORBOT ER III
 */

#ifndef SCORBOT_ER_III_H
#define SCORBOT_ER_III_H
#include "tipos.h"
#include "motor.h"

void modo_demo(void);
void programacion_manual(void);

void base_HOME(void);
void hombro_HOME(void);
void codo_HOME(void);
void inclinacion_HOME(void);
void rotacion_HOME(void);
void SCORBOT_HOME(void);

void mover_base(int16_t posicion);
void mover_hombro(int16_t posicion);
void mover_codo(int16_t posicion);
void inclinar_pinza(int16_t posicion);
void rotar_pinza(int16_t posicion);
void abrir_cerrar_pinza(int16_t posicion);
void abrir_pinza(void);
void cerrar_pinza(void);

void esperar_recibir_movimientos(void);
void devolver_grados_reales(void);
int16_t convertir_pulsos_grados(Motor_TypeDef *motor , int16_t pulsos);

#endif /*SCORBOT_ER_III_H*/