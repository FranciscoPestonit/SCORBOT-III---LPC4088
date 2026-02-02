/*******************************************************************************
 * \file    EEPROM_lpc40xx.h
 *
 * \brief   Funciones para manejar el módulo EEPROM
 */
 
 
 #include "EEPROM_lpc40xx.h"
 #include "tipos.h"
 
 void eeprom_inicializar(void){
 
 //Está activado el POWER por defecto tras un reset
//LPC_EEPROM->PWRDWN = (1u << 0); //ESTA INSTRUCCION LO APAGA
 
//Configurar los pulsos de reloj de la EEPROM
   
  LPC_EEPROM->WSTATE  = (2u << 0) | (6u << 8) | (4u << 16);
//Para una frecuencia de CPU de 120MHz
  LPC_EEPROM->CLKDIV = 319;          //Si fueran 60 MHz, sería 159
//Nos aseguramos de que tenemos la EEPROM con alimentación
   LPC_EEPROM->PWRDWN = 0;

 }
 
 void eeprom_escribir_16bit(uint8_t pagina, int16_t dato){
 
  if(pagina >63){return;}

 //WRITE ADDRESS REGISTER
 LPC_EEPROM->ADDR = (pagina << 6); //Dirección de la primera página con una operación de x(64)

 //WRITE COMMAND REGISTER
 LPC_EEPROM->CMD = 4; //100 16 bit write
 
 //WRITE DATA REGISTER
 LPC_EEPROM->WDATA = dato;
 
 eeprom_esperar_rw();
   
 //WRITE ADDRESS REGISTER
 LPC_EEPROM->ADDR = (pagina << 6);
   
 //ERASE/PROGRAM COMMAND
 LPC_EEPROM->CMD = 6;
 
 eeprom_esperar_programa();
 }
 
 int16_t eeprom_leer_16bit(uint8_t pagina){
 
 if(pagina >63){return 0;}
   
 int16_t dato_leido;
   
 //WRITE ADDRESS REGISTER
 LPC_EEPROM->ADDR = (pagina << 6);
 
 //WRITE COMMAND REGISTER
 LPC_EEPROM->CMD = 1; //001 16 bit read
 
 //READ DATA REGISTER
 dato_leido = LPC_EEPROM->RDATA;
   
 return dato_leido;
 }
 
 void eeprom_esperar_programa(void){
 
  while (!(LPC_EEPROM->INT_STATUS & (1u << 28))){}
  LPC_EEPROM->INT_CLR_STATUS = (1u << 28);
 
 }
 
 void eeprom_esperar_rw(void){
 
 while (!(LPC_EEPROM->INT_STATUS & (1u << 26))){}
 LPC_EEPROM->INT_CLR_STATUS = (1u << 26); 
 
 
 }