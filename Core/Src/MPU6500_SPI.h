/*
 * MPU6500_SPI.h
 *
 *  Created on: 31 de jul de 2023
 *      Author: Fagner
 */

#ifndef MPU6500_SPI_H_
#define MPU6500_SPI_H_
#include "Utility.h"
#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "math.h"


//Defini��es dos endere�os dos registradores
//Todos os registradores iniciam em 0x00, exceto: (WHO_AM_I = 0x70), (PWR_MGMT_1 = 0x01) e cancelamento de offsets do aceler�metro
#define	XG_OFFSET_H			(uint8_t)0x13	//cancelamento de offsets do girosc�pio
#define XG_OFFSET_L			(uint8_t)0x14	//
#define YG_OFFSET_H			(uint8_t)0x15	//
#define	YG_OFFSET_L			(uint8_t)0x16	//
#define	ZG_OFFSET_H			(uint8_t)0x17	//
#define ZG_OFFSET_L			(uint8_t)0x18	//
#define	SMPLRT_DIV			(uint8_t)0x19	//sample rate divider
#define	CONFIG				(uint8_t)0x1A	//configura��es gerais
#define	GYRO_CONFIG			(uint8_t)0x1B	//configura��es do girosc�pio
#define	ACCEL_CONFIG		(uint8_t)0x1C	//configura��es do aceler�metro
#define	ACCEL_CONFIG2		(uint8_t)0x1D	//configura��es do aceler�metro
#define	WOM_THR				(uint8_t)0x1F	//configura��es de threshold do WakeOnMotion
#define INT_PIN_CFG			(uint8_t)0x37	//configura��es do pino de interrup��o
#define	INT_ENABLE			(uint8_t)0x38	//habilita��o de interrup��es
#define	INT_STATUS			(uint8_t)0x3A	//status das interrup��es
#define DATA_ARRAY_POINTER	(uint8_t)0x3B	//ponteiro para o in�cio dos registradores de dados do sensor
#define	SIGNAL_PATH_RESET	(uint8_t)0x68	//reset do caminho de dados digitais
#define	ACCEL_INTEL_CTRL	(uint8_t)0x69	//controle de inteligencia do aceler�metro
#define USER_CTRL			(uint8_t)0x6A	//controles do usu�rio
#define	PWR_MGMT_1			(uint8_t)0x6B	//gerenciamento de energia
#define	PWR_MGMT_2			(uint8_t)0x6C	//gerenciamento de energia
#define	WHO_AM_I			(uint8_t)0x75	//ID do MPU-6500 (ID = 0x70)
#define XA_OFFSET_H			(uint8_t)0x77	//cancelamento de offsets do aceler�metro
#define	XA_OFFSET_L			(uint8_t)0x78	//
#define	YA_OFFSET_H			(uint8_t)0x7A	//
#define	YA_OFFSET_L			(uint8_t)0x7B	//
#define	ZA_OFFSET_H			(uint8_t)0x7D	//
#define	ZA_OFFSET_L			(uint8_t)0x7E	//

//Defini��es dos bits dos registradores
#define	DEVICE_RESET		(uint8_t)(1 << 7)	//bit 7 do registrador PWR_MGMT_1
#define	I2C_IF_DIS			(uint8_t)(1 << 4)	//bit 4 do registrador USER_CTRL
#define	SIG_COND_RST		(uint8_t)(1 << 0)	//bit 0 do registrador USER_CTRL
#define	RAW_RDY_EN			(uint8_t)(1 << 0)	//bit 0 do registrador INT_ENABLE
#define	WOM_EN				(uint8_t)(1 << 6)	//bit 6 do registrador INT_ENABLE
#define	ACCEL_INTEL_EN		(uint8_t)(1 << 7)	//bit 7 do registrador ACCEL_INTEL_CTRL
#define	ACCEL_INTEL_MODE	(uint8_t)(1 << 6)	//bit 6 do registrador ACCEL_INTEL_CTRL

//Defini��es globais do sensor
#define RAW_ACCEL_X_OFFSET	0	//valores de cancelamento de offsets do aceler�metro (1024 LSB = 1g)
#define RAW_ACCEL_Y_OFFSET	0
#define RAW_ACCEL_Z_OFFSET	32
#define RAW_GYRO_X_OFFSET	92	//valores de cancelamento de offsets do girosc�pio (32,768 LSB = 1�/s)
#define RAW_GYRO_Y_OFFSET	376
#define RAW_GYRO_Z_OFFSET	-11
#define	accelScalingFactor	(float)2/32768		//fator de escala do aceler�metro �2g
#define	gyroScalingFactor	(float)250/32768	//fator de escala do girosc�pio �250 �/s

//Prot�tipos de fun��es de acesso ao sensor MPU-6500
void SPI2_Init(void);												//inicializa��o da interface SPI2 para opera��o com MPU6500
void MPU6500_Reset(void);											//reset do sensor
void MPU6500_Config(void);											//configura��es iniciais do sensor
void MPU6500_Get_Offset_Biases(void);								//retorna os valores de offset com o sensor estacion�rio. A fun��o s� deve ser chamada ap�s MPU6500_Config()
void MPU6500_Offset_Cancellation(void);								//configura os valores de cancelamento de offset nos registradores do sensor
uint8_t Read_Data(uint8_t address);									//leitura de um byte
void Read_MData(uint8_t address, uint8_t Nbytes, uint8_t *Data);	//leitura de m�ltiplos bytes
void Write_Data(uint8_t address, uint8_t data);						//escrita de um byte


//Declara��o de fun��es de acesso ao sensor MPU-6500
//inicializa��o da interface SPI2
void SPI2_Init(void);
//Reset do sensor
void MPU6500_Reset(void);
//Configura��o inicial do sensor
void MPU6500_Config(void);

//Retorna os valores de offset com o sensor estacion�rio
void MPU6500_Get_Offset_Biases(void);

//Configura os valores de offset nos registradores de cancelamento de offset do sensor
void MPU6500_Offset_Cancellation(void);
//Leitura de um byte
uint8_t Read_Data(uint8_t address);

//Leitura de m�ltiplos bytes
void Read_MData(uint8_t address, uint8_t Nbytes, uint8_t *Data);

//Escrita de um byte
void Write_Data(uint8_t address, uint8_t data);

#endif /* MPU6500_SPI_H_ */
