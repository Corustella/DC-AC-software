/*
 * key.c
 *
 *  Created on: Jun 2, 2025
 *      Author: Spencer
 */
#include "key.h"

int getKeyValue(void)
{

    int key_value = 0;
    int inputs=0;

    GPIOD->BSRR = 0x04900008;// Set:PD3,Reset:PD4 PD7 PD10
    Key_Delay();
    inputs=((GPIOE->IDR & 0x0000001a)>>(1U))|((GPIOD->IDR & 0x00000800)>>(10U));
    key_value = inputs;


    GPIOD->BSRR = 0x04880010;// Set:PD4,Reset:PD3 PD7 PD10
    Key_Delay();
    inputs=((GPIOE->IDR & 0x0000001a)>>(1U))|((GPIOD->IDR & 0x00000800)>>(10U));
    key_value = key_value+(inputs<<(4U));

    GPIOD->BSRR = 0x04180080;// Set:PD7,Reset:PD3 PD4 PD10
    Key_Delay();
    inputs=((GPIOE->IDR & 0x0000001a)>>(1U))|((GPIOD->IDR & 0x00000800)>>(10U));
    key_value = key_value+(inputs<<(8U));

    GPIOD->BSRR = 0x00980400;// Set:PD10,Reset:PD3 PD4 PD7
    Key_Delay();
    inputs=((GPIOE->IDR & 0x0000001a)>>(1U))|((GPIOD->IDR & 0x00000800)>>(10U));
    key_value = key_value+(inputs<<(12U));

    key_value=__builtin_ctz(key_value)+1;//寻找（是1的最低的位数）末尾0的个数

    if (key_value<17 )
    	return key_value; // 返回按键
    else return 0;// 没有按下，返回0
}
void delay_us(uint32_t udelay)
{
  uint32_t startval,tickn,delays,wait;

  startval = SysTick->VAL;//记录函数调用时 SysTick->VAL 的初始值
  tickn = HAL_GetTick();//记录当前的毫秒时间戳（防止跨越毫秒边界出错）
  delays =udelay * 168; //将微秒转换为 SysTick 的时钟周期数

  if(delays > startval)//当前毫秒剩余周期数不足
    {
      while(HAL_GetTick() == tickn){}
      wait = 168000 + startval - delays;// 计算新周期的终止值
      while(wait < SysTick->VAL){}// 等待计数器递减到目标值
    }
  else
    {
      wait = startval - delays;
      while(wait < SysTick->VAL && HAL_GetTick() == tickn){}
    }
}


