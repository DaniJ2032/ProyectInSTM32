/*
 * structrForFrames.h
 *
 *  Created on: Oct 13, 2023
 *      Author: DaniBrem
 */

#ifndef INC_STRUCTRFORFRAMES_H_
#define INC_STRUCTRFORFRAMES_H_

// Declaracion de estructuras y uniones
typedef struct {
  // Byte de caebcera.
  uint8_t start;
  // Contador.
  uint8_t count;
  // 8 entradas analógicas (12 bits).
  uint16_t inA1;
  uint16_t inA2;
  uint16_t inA3;
  uint16_t inA4;
  uint16_t inA5;
  uint16_t inA6;
  uint16_t inA7;
  uint16_t inA8;
  // 2 salidas analógicas.
  uint16_t outA1;
  uint16_t outA2;
  // 8 entradas digitales.
  uint8_t insDig;
  // 8 salidas digitales.
  uint8_t outsDig;

} frame_t; //Fin de struct

typedef union
{
  frame_t frameEntrada;
  char    frameEntradaChar[24];
} charFrame_t; //Fin de union


#endif /* INC_STRUCTRFORFRAMES_H_ */
