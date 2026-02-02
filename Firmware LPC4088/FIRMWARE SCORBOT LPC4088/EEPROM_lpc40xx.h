/*******************************************************************************
 * \file    EEPROM_lpc40xx.h
 *
 * \brief   Funciones para manejar el módulo EEPROM
 */
 
 
#ifndef EEPROM_H
#define EEPROM_H

#include "tipos.h"

#define PAGE0    0
#define PAGE1    1
#define PAGE2    2
#define PAGE3    3
#define PAGE4    4
#define PAGE5    5
#define PAGE6    6
#define PAGE7    7
#define PAGE8    8
#define PAGE9    9
#define PAGE10   10
#define PAGE11   11
#define PAGE12   12
#define PAGE13   13
#define PAGE14   14
#define PAGE15   15
#define PAGE16   16
#define PAGE17   17
#define PAGE18   18
#define PAGE19   19
#define PAGE20   20
#define PAGE21   21
#define PAGE22   22
#define PAGE23   23
#define PAGE24   24
#define PAGE25   25
#define PAGE26   26
#define PAGE27   27
#define PAGE28   28
#define PAGE29   29
#define PAGE30   30
#define PAGE31   31
#define PAGE32   32
#define PAGE33   33
#define PAGE34   34
#define PAGE35   35
#define PAGE36   36
#define PAGE37   37
#define PAGE38   38
#define PAGE39   39
#define PAGE40   40
#define PAGE41   41
#define PAGE42   42
#define PAGE43   43
#define PAGE44   44
#define PAGE45   45
#define PAGE46   46
#define PAGE47   47
#define PAGE48   48
#define PAGE49   49
#define PAGE50   50
#define PAGE51   51
#define PAGE52   52
#define PAGE53   53
#define PAGE54   54
#define PAGE55   55
#define PAGE56   56
#define PAGE57   57
#define PAGE58   58
#define PAGE59   59
#define PAGE60   60
#define PAGE61   61
#define PAGE62   62
#define PAGE63   63

void eeprom_inicializar(void);
void eeprom_escribir_16bit(uint8_t pagina, int16_t dato);
void eeprom_esperar_programa(void);
void eeprom_esperar_rw(void);
int16_t eeprom_leer_16bit(uint8_t pagina);

#endif  /* EEPROM_H */