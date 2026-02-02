/**
 * @file    uart_lpc40xx.c
 * @brief   Funciones de manejo de las UARTs del LPC40xx.
 *
 * @author      Alejandro Lara Doña - alejandro.lara@uca.es | Eduardo Romero
 * @date        2014/2025
 * @version     2.0
 *
 * @copyright   GNU General Public License version 3 or later
 */

#include "uart_lpc40xx.h"
#include <LPC407x_8x_177x_8x.h>
#include <math.h>
#include "error.h"
#include "iocon_lpc40xx.h"

// ===== UART - Funciones privadas =====
static float32_t uart_calcular_baudrate(LPC_UART_TypeDef *uart_regs, uint32_t baudrate);


/**
 * @brief     Inicializa una UART del LPC40xx.
 * @ingroup  UART
 *
 * @param[in]   uart_regs               Ptr. al bloque de registros de la UART.
 * @param[in]   baudrate                Velocidad en baudios requerida.
 * @param[in]   numero_bits_datos       Bits de datos a usar. De 5 a 8.
 * @param[in]   tipo_paridad            Paridad a usar (NINGUNA, IMPAR o PAR).
 * @param[in]   numero_bits_stop        Bits de stop a usar (1 ó 2).
 * @param[in]   puerto_txd              Puerto que se desea para la señal TXD.
 * @param[in]   mascara_pin_txd         Pin que se desea para la señal TXD.
 * @param[in]   puerto_rxd              Puerto que se desea para la señal RXD.
 * @param[in]   mascara_pin_rxd         Pin que se desea para la señal RXD.
 * @param[out]  baudrate_real_obtenido  Puntero a float32_t donde almacenar el mejor baudrate que
 *                                      se pudo ajustar. Si es NULL no se usa.
 *
 * @note  La razón de especificar los puertos y los pines mediante punteros a los registros GPIO
 * y máscaras de pin en lugar de números de puerto y números de pin es conseguir que la forma de
 * especificar un pin sea igual que la usada en los módulos gpio_lpc40xx e iocon_lpc40xx. Esto
 * permite usar los mismos símbolos PUERTOx y PINx definidos en gpio_lpc40xx.h.
 */
void uart_inicializar(LPC_UART_TypeDef *uart_regs,uart_baudrate_t baudrate,
                      uart_bits_datos_t numero_bits_datos, uart_paridad_t tipo_paridad,
                      uart_bits_stop_t numero_bits_stop,
                      LPC_GPIO_TypeDef *puerto_txd, uint32_t mascara_pin_txd,
                      LPC_GPIO_TypeDef *puerto_rxd, uint32_t mascara_pin_rxd,
                      float32_t *baudrate_real_obtenido) {

  float32_t baudrate_calculado;

  // Comprobar que los parámetros son correctos
  // Los números de puerto y pin se comprueban en las funciones de configuración de pines
  ASSERT(uart_regs == UART0 || uart_regs == UART1 || uart_regs == UART2 ||
         uart_regs == UART3 || uart_regs == UART4, "Puntero a registros de UART incorrecto.");

  ASSERT(numero_bits_datos >= UART_BITS_DATOS_5 && numero_bits_datos <= UART_BITS_DATOS_8,
         "Numero de bits de datos seleccionado incorrecto.");

  ASSERT(tipo_paridad == UART_PARIDAD_NINGUNA || tipo_paridad == UART_PARIDAD_IMPAR ||
         tipo_paridad == UART_PARIDAD_PAR     || tipo_paridad == UART_PARIDAD_UNO   ||
         tipo_paridad == UART_PARIDAD_CERO, "Paridad seleccionada incorrecta.");

  ASSERT(numero_bits_stop == UART_BITS_STOP_1 || numero_bits_stop == UART_BITS_STOP_2,
         "Numero de bits de stop seleccionado incorrecto.");

  // Activar la alimentación de la UART correspondiente
  if (uart_regs == UART0) {
    LPC_SC->PCONP |= 1u << 3;

  } else if (uart_regs == UART1) {
    LPC_SC->PCONP |= 1u << 4;

  } else if (uart_regs == UART2) {
    LPC_SC->PCONP |= 1u << 24;

  } else if (uart_regs == UART3) {
    LPC_SC->PCONP |= 1u << 25;

  } else if (uart_regs == UART4) {
    LPC_SC->PCONP |= 1u << 8;
  }

  // Ajustar la velocidad de comunicación en baudios solicitada (página 508-509 del manual)
  baudrate_calculado = uart_calcular_baudrate(uart_regs, baudrate);

  if (baudrate_real_obtenido != NULL) {
    *baudrate_real_obtenido = baudrate_calculado;
  }

  // Configurar el resto de parámetros en el registro LCR
  uart_regs->LCR = (numero_bits_datos << UART_LCR_BITS_DATO) |
                   (numero_bits_stop << UART_LCR_BITS_STOP)  | (tipo_paridad << UART_LCR_PARIDAD);

  // Activar y limpiar las FIFOs
  uart_regs->FCR = UART_FCR_FIFOEN | UART_FCR_RXFIFORES;

  // Configurar los pines indicados
  if (uart_regs == UART0) {
    iocon_configurar_pin(puerto_txd, mascara_pin_txd, U0_TXD, IOCON_NO_PULL_UP_NO_PULL_DOWN);
    iocon_configurar_pin(puerto_rxd, mascara_pin_rxd, U0_RXD, IOCON_PULL_UP);

  } else if (uart_regs == UART1) {
    iocon_configurar_pin(puerto_txd, mascara_pin_txd, U1_TXD, IOCON_NO_PULL_UP_NO_PULL_DOWN);
    iocon_configurar_pin(puerto_rxd, mascara_pin_rxd, U1_RXD, IOCON_PULL_UP);

  } else if (uart_regs == UART2) {
    iocon_configurar_pin(puerto_txd, mascara_pin_txd, U2_TXD, IOCON_NO_PULL_UP_NO_PULL_DOWN);
    iocon_configurar_pin(puerto_rxd, mascara_pin_rxd, U2_RXD, IOCON_PULL_UP);

  } else if (uart_regs == UART3) {
    iocon_configurar_pin(puerto_txd, mascara_pin_txd, U3_TXD, IOCON_NO_PULL_UP_NO_PULL_DOWN);
    iocon_configurar_pin(puerto_rxd, mascara_pin_rxd, U3_RXD, IOCON_PULL_UP);

  } else {
    iocon_configurar_pin(puerto_txd, mascara_pin_txd, U4_TXD, IOCON_NO_PULL_UP_NO_PULL_DOWN);
    iocon_configurar_pin(puerto_rxd, mascara_pin_rxd, U4_RXD, IOCON_PULL_UP);
  }
}

/**
 * @brief     Transmite un dato mediante la UART.
 * @ingroup  UART
 *
 * @param[in]   uart_regs   Puntero al bloque de registros de la UART.
 * @param[in]   dato        Dato a transmitir.
 */
void uart_transmitir_dato(LPC_UART_TypeDef *uart_regs, uint8_t dato) {

  ASSERT(uart_regs == UART0 || uart_regs == UART1 || uart_regs == UART2 ||
         uart_regs == UART3 || uart_regs == UART4, "Puntero a registros de UART incorrecto.");

  while (!(uart_regs->LSR & UART_LSR_THRE)) {;}  // Esperar a que la FIFO de transmisión esté libre

  uart_regs->THR = dato;
}


void uart_transmitir_int16(LPC_UART_TypeDef *uart_regs, int16_t valor)
{
    uint8_t lsb = (uint8_t)(valor & 0xFF);
    uint8_t msb = (uint8_t)((valor >> 8) & 0xFF);

    uart_transmitir_dato(uart_regs, lsb);
    uart_transmitir_dato(uart_regs, msb);
}


/**
 * @brief     Indica si hay al menos un dato en el FIFO de recepción de la UART.
 * @ingroup  UART
 *
 * @param[in]   uart_regs   Puntero al bloque de registros de la UART.
 *
 * @retval      FALSE - Ningún dato disponible.
 * @retval      TRUE  - Al menos un dato disponible.
 */
bool_t uart_hay_dato_disponible(const LPC_UART_TypeDef *uart_regs) {

  ASSERT(uart_regs == UART0 || uart_regs == UART1 || uart_regs == UART2 ||
         uart_regs == UART3 || uart_regs == UART4, "Puntero a registros de UART incorrecto.");

  if (uart_regs->LSR & UART_LSR_RDR) {
    return TRUE;
  }

  return FALSE;
}

/**
 * @brief     Leer un dato del FIFO de recepción de una UART. Debe ser llamada si se sabe que hay 
 * al menos un dato en la FIFO.
 * @ingroup  UART
 *
 * @param[in]   uart_regs   Puntero al bloque de registros de la UART.
 *
 * @return      Dato obtenido del FIFO de la UART.
 */
uint8_t uart_leer_dato(const LPC_UART_TypeDef *uart_regs) {

  ASSERT(uart_regs == UART0 || uart_regs == UART1 || uart_regs == UART2 ||
         uart_regs == UART3 || uart_regs == UART4, "Puntero a registros de UART incorrecto.");

  return uart_regs->RBR;
}

/**
 * @brief     Espera a recibir un dato a través de la UART.
 * @ingroup  UART
 *
 * @param[in]   uart_regs   Puntero al bloque de registros de la UART.
 *
 * @return      Dato recibido.
 */
uint8_t uart_esperar_recibir_dato(const LPC_UART_TypeDef *uart_regs) {

  ASSERT(uart_regs == UART0 || uart_regs == UART1 || uart_regs == UART2 ||
         uart_regs == UART3 || uart_regs == UART4, "Puntero a registros de UART incorrecto.");

  while (!(uart_regs->LSR & UART_LSR_RDR)) {;} // Esperar a que haya un dato disponible
  return uart_regs->RBR;
}

int16_t uart_recibir_int16(const LPC_UART_TypeDef *uart_regs)
{
    uint8_t lsb = uart_esperar_recibir_dato(uart_regs);
    uint8_t msb = uart_esperar_recibir_dato(uart_regs);

    return (int16_t)((msb << 8) | lsb);
}

/**
 * @brief     Transmite una cadena de caracteres mediante la UART.
 * @ingroup  UART
 *
 * @param[in]   uart_regs   Puntero al bloque de registros de la UART.
 * @param[in]   cadena      Puntero a la cadena a transmitir.
 */
void uart_transmitir_cadena(LPC_UART_TypeDef *uart_regs, const char *cadena) {

  ASSERT(uart_regs == UART0 || uart_regs == UART1 || uart_regs == UART2 ||
         uart_regs == UART3 || uart_regs == UART4, "Puntero a registros de UART incorrecto.");

  ASSERT(*cadena != 0, "Cadena vacia.");

  while (*cadena != 0) {
    uart_transmitir_dato(uart_regs, (char) *cadena);
    cadena++;
  }
}

/**
 * @brief     Recibe una cadena de caracteres mediante la UART.
 * @ingroup  UART
 *
 * @details   La recepción de la cadena terminará cuando se reciba el carácter nueva línea '\n').
 * La función descarta todos los caracteres no imprimibles (mirar la tabla de códigos ASCII) y sólo
 * almacena en el buffer los (tamano_buffer - 1) primeros caracteres que se reciban y descartará el
 * resto. No se trata el carácter de borrado (backspace, ASCII = 0x08), así que los caracteres que
 * se reciban y almacen en la cadena no pueden borrarse después.
 *
 * @param[in]   uart_regs       Puntero al bloque de registros de la UART.
 * @param[out]  ptr_buffer      Puntero al buffer donde se almacenará la cadena recibida. La cadena
 *                              quedará terminada con un carácter nulo.
 * @param[in]   tamano_buffer   Longitud del buffer en bytes. La función sólo almacenará en el
 *                              buffer los (tamano_buffer - 1) primeros caracteres que se reciban.
 * @warning     El tamaño del buffer debe ser al menos 1 para tener espacio para el terminador
 * (pero, si es 1, solo aceptará la pulsacón de ENTER para finalizar la entrada).
 */
void uart_recibir_cadena(const LPC_UART_TypeDef *uart_regs, char *ptr_buffer,
                         uint32_t tamano_buffer) {

  ASSERT(uart_regs == UART0 || uart_regs == UART1 || uart_regs == UART2 ||
         uart_regs == UART3 || uart_regs == UART4, "Puntero a registros de UART incorrecto.");

  ASSERT(ptr_buffer != NULL, "ptr_buffer no puede ser nulo.");

  while (1) {
    char caracter_recibido = uart_esperar_recibir_dato(uart_regs);

    if (caracter_recibido >= ' ' && tamano_buffer > 1) {
      *ptr_buffer = caracter_recibido;
      ptr_buffer++;
      tamano_buffer--;
    }

    if (caracter_recibido == '\n') {
      *ptr_buffer = 0;
      return;
    }
  }
}

/**
 * @brief     Habilita/deshabilita las interrupciones al recibir un nuevo dato en la UART indicada.
 * @ingroup  UART
 *
 * @param[in]   uart_regs   Puntero al bloque de registros de la UART.
 * @param[out]  estado      Si es TRUE, activa las interrupciones y en caso contrario las desactiva.
 */
void uart_habilitar_interrupciones_dato_recibido(LPC_UART_TypeDef *uart_regs, bool_t estado) {
  if (estado) {
    uart_regs->IER |= UART_IER_RBR;
  } else {
    uart_regs->IER &= ~UART_IER_RBR;
  }
}

/**
 * @brief   Función privada para calcular y ajustar los registros de preescala del reloj para
 * configurar el baudrate indicado para la transmisión.
 * @ingroup  UART
 * @private 
 *
 * @details Se utiliza el algoritmo propuesto por NXP en la página 509. (Error relativo < 1.1%)
 *
 * La mejor configuración de DIVADDVAL y MULVAL se encontrará probando todos los valores
 * posibles de DIVADDVAL / MULVAL y seleccionando aquellos que hacen mínimo el error relativo:
 *
 * \f[
 *    Baudrate_{calculado} = \frac{PeripheralClock}{16*DLMDLL*(1+DIVADDVAL/MULVAL)}
 * \f]
 * \f[
 *    Error_{relativo} = \frac{|Baudrate_{deseado}-Baudrate_{calculado}|}{Baudrate_{deseado}} = 
 *    |1-\frac{Baudrate_{calculado}}{Baudrate_{deseado}}|
 * \f]
 * \f[
 *    Error_{relativo} = |1-\frac{PeripheralClock}{Baudrate_{deseado}*16*DLMDLL*
 *    (1+\frac{DIVADDVAL}{MULVAL})}|
 * \f]
 *
 * Hay ciertas condiciones que hay que cumplir:
 *
 * - Si el divisor fraccional (FDR) está activo y DLM = 0, entonces DLL debe ser mayor que 2.
 * - El valor de MULVAL y DIVADDVAL deben cumplir las siguientes condiciones:
 *   - 1 <= MULVAL <= 15
 *   - 0 <= DIVADDVAL <= 14
 *   - DIVADDVAL < MULVAL
 *
 * Si DIVADDVAL = 0, se deshabilita el divisor fraccional.
 *
 * @return  Baudrate real obtenido tras el ajuste.
 *
 * @warning Se modifican los registros DLL, DLM y FDR.
 *
 * @see     Página 509 del LPC408x/407x User manual (UM10562)
 */

static float32_t uart_calcular_baudrate(LPC_UART_TypeDef *uart_regs, uint32_t baudrate) {

  uint8_t mejor_divaddval = 0, mejor_mulval = 1;

  // Constante que se utiliza muchas veces en los cálculos
  const float32_t baudrate_16 = (16.0f * (float32_t) baudrate);

  // Primero, calcular el divisor de baudrate suponiendo DIVADDVAL = 0 y MULVAL = 1
  float32_t dlmdll_f = (float32_t) PeripheralClock / baudrate_16;

  uint32_t dlmdll = (uint32_t) dlmdll_f; // Convertir a entero

  // Si es entero, divaddval = 0, mulval = 1 y se puede pasar a configurar ya los divisores
  if (dlmdll_f == dlmdll) {
    // El divisor entero dlmdll tiene que ser mayor que cero
    if (dlmdll == 0) {
      dlmdll = 1;
    }

  } else {
    // Calcular la parte fraccionaria del divisor

    // 1 - Calculamos el error actual suponiendo divaddval = 0 y mulval = 1
    float32_t mejor_error_relativo = fabs(1.0f - ((float32_t) PeripheralClock /
                                     (baudrate_16 * (float32_t) dlmdll)));

    // 2 - Buscar los valores de divaddval y mulval que hacen mínimo el error
    for (uint32_t divisor_entero = dlmdll - 1; divisor_entero <= dlmdll + 1; divisor_entero++) {

      for (uint8_t mulval = 1; mulval < 16; mulval++) {

        for (uint8_t divaddval = 1; divaddval < mulval; divaddval++) {

          float32_t error_relativo = fabs((1.0f - ((float32_t) PeripheralClock /
                                     (baudrate_16 * (float32_t) divisor_entero *
                                     (1.0f + (float32_t) divaddval/ (float32_t) mulval)))));

          if (error_relativo < mejor_error_relativo) {
            dlmdll = divisor_entero;
            mejor_divaddval = divaddval;
            mejor_mulval = mulval;
            mejor_error_relativo = error_relativo;
          }
          if (mejor_error_relativo == 0) {
            break;
          }
        }  // Bucle divaddval

        if (mejor_error_relativo == 0) {
          break;
        }
      }  // Bucle mulval
      if (mejor_error_relativo == 0) {
        break;
      }
    }  // Bucle divisor_entero
  }


  // Activar el acceso a los registros del divisor del reloj (DLM y DLL)
  uart_regs->LCR |= UART_LCR_ACCESO_DIVISOR_LATCH;

  // Escribir los valores calculados en DLM, DLL y FDR
  uart_regs->DLM = dlmdll >> 8;
  uart_regs->DLL = dlmdll & 0xFF;
  uart_regs->FDR = (mejor_mulval << 4) | mejor_divaddval;

  uart_regs->LCR &= ~UART_LCR_ACCESO_DIVISOR_LATCH;

  // Devolver el baudrate real obtenido
  return (float32_t) PeripheralClock /
         (16.0f * dlmdll *(1.0f + (float32_t) mejor_divaddval / (float32_t) mejor_mulval));
}
