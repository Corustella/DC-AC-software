/*
 * key.h
 *
 *  Created on: Jun 2, 2025
 *      Author: Spencer
 */

#ifndef KEY_KEY_H_
#define KEY_KEY_H_

#include "main.h"
#include "stm32f4xx.h"
#define Key_Delay()  delay_us(3)

int getKeyValue(void);
void delay_us(uint32_t udelay);

#endif /* KEY_KEY_H_ */
