/*
 * Utility.h
 *
 *  Created on: 3 de jan de 2024
 *      Author: Prof. Dr. Fagner de Araujo Pereira
 */

#ifndef UTILITY_H_
#define UTILITY_H_
#include "stm32f4xx_hal.h"
//Definições Globais

//definições dos números dos pinos GPIO
#define pin_0	((uint8_t) 0)
#define pin_1	((uint8_t) 1)
#define pin_2	((uint8_t) 2)
#define pin_3	((uint8_t) 3)
#define pin_4	((uint8_t) 4)
#define pin_5	((uint8_t) 5)
#define pin_6	((uint8_t) 6)
#define pin_7	((uint8_t) 7)
#define pin_8	((uint8_t) 8)
#define pin_9	((uint8_t) 9)
#define pin_10	((uint8_t) 10)
#define pin_11	((uint8_t) 11)
#define pin_12	((uint8_t) 12)
#define pin_13	((uint8_t) 13)
#define pin_14	((uint8_t) 14)
#define pin_15	((uint8_t) 15)

//definições dos modos de operação de um pino GPIO
#define	INPUT		((uint8_t) 0b00)	//modo de entrada digital
#define	OUTPUT		((uint8_t) 0b01)	//modo de saída digital
#define	ALTERNATE	((uint8_t) 0b10)	//modo de função alternativa
#define	ANALOG		((uint8_t) 0b11)	//modo analógico

//definições dos tipos de saída de um pino GPIO
#define	PUSH_PULL	((uint8_t) 0)	//saída push-pull
#define	OPEN_DRAIN	((uint8_t) 1)	//saída open-drain

//definições para o nível lógico de saída de um pino
#define	LOW		((uint8_t) 0)	//nível lógico baixo
#define	HIGH	((uint8_t) 1)	//nível lógico alto

//definições dos modos de operação dos resistores de pull-up e pull-down
#define	PULL_UP		((uint8_t) 0b01)	//resistor de pull-up
#define	PULL_DOWN	((uint8_t) 0b10)	//resistor de pull-down


//Declarações de funções úteis

//Funções de configuração do sistema de clock do STM32
void Utility_Init(void);		//inicialização de funções da biblioteca
void Configure_Clock(void);		//configuração do sistema de clock

//Funções de timers e temporização
void TIM2_Setup(void);				//configuração do Timer2 como base de tempo de 1us
void Delay_us(uint32_t delay);		//atraso em us
void Delay_ms(uint32_t delay);		//atraso em ms

//Funções de manipulação de GPIO
void GPIO_Clock_Enable(GPIO_TypeDef* GPIOx);								//habilita o clock de um GPIO
void GPIO_Pin_Mode(GPIO_TypeDef* GPIOx, uint8_t pin, uint8_t mode);			//configura o modo de operação de um pino de um GPIO
void GPIO_Output_Mode(GPIO_TypeDef* GPIOx, uint8_t pin, uint8_t mode);		//configura o tipo de saída de um pino de um GPIO
void GPIO_Write_Pin(GPIO_TypeDef* GPIOx, uint8_t pin, uint8_t level);		//escreve um nível lógico em um pino de um GPIO
void GPIO_Toggle_Pin(GPIO_TypeDef* GPIOx, uint8_t pin);						//inverte o nível lógico em um pino de um GPIO
void GPIO_Write_Port(GPIO_TypeDef* GPIOx, uint16_t value);					//escreve um valor numa porta GPIO
void GPIO_Resistor_Enable(GPIO_TypeDef* GPIOx, uint8_t pin, uint8_t mode);	//habilita os resistores de pull-up ou pull-down
uint8_t GPIO_Read_Pin(GPIO_TypeDef* GPIOx, uint8_t pin);					//lê e retorna o nível lógico em um pino de um GPIO
uint16_t GPIO_Read_Port(GPIO_TypeDef* GPIOx);								//lê e retorna os níveis lógicos de uma porta GPIO

//Funções de periféricos de comunicação
void USART1_Init(void);				//configuração da USART1 para debug com printf

//Funções de geração de números aleatórios
uint32_t Random_Number(void);		//retorna um número aleatório de 32 bits


//Funções de configuração do sistema de clock do STM32
//Inicialização de funções da biblioteca

#endif /* UTILITY_H_ */
