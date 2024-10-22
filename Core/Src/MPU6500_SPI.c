#include "MPU6500_SPI.h"

//*********************************************************************
// Função para inicialização da interface SPI2
//*********************************************************************
void SPI2_Init(void)
{
	//configurando os pinos da interface SPI2 no modo de fun��o alternativa
	//PD12 como CS (NSS) - opera  o como GPIO
	//PB10 como SCK
	//PB14 como MISO
	//PB15 como MOSI
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;// habilita o clock do GPIOB
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;//habilita o clock do GPIOD
	GPIOD->ODR |= 1 << 12;										//pino PD12 inicialmente em n�ve alto
	GPIOB->MODER |= (0b10 << 30) | (0b10 << 28) | (0b10 << 20);	//pinos PB15, PB14 e PB10 como fun��o alternativa
	GPIOD->MODER |=  (0b01 << 24); // PD12 como GPIO de sa�da NCS.
	GPIOB->AFR[1] |= (0b0101 << 28) | (0b0101 << 24) | (0b0101 << 8);			//fun  o alternativa 5 (SPI2)

	//configurando a interface SPI2
	RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;			//habilita o clock da interface SPI2

	//pino SS controlado por software, baud rate em 656.25 kHz, configura o modo master e habilita a interface
	SPI2->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI | (SPI_CR1_BR_2 | SPI_CR1_BR_0) | SPI_CR1_MSTR | SPI_CR1_SPE;
	}

//*********************************************************************
//Reset do sensor MPU
//*********************************************************************
void MPU6500_Reset(void)
{
	//Resetando o sensor MPU-6500
	Write_Data(PWR_MGMT_1, DEVICE_RESET);		//reset do sensor
	Delay_ms(100);								//aguarda 100ms
	Write_Data(SIGNAL_PATH_RESET, 0b00000111);	//reset dos filtros digitais
	Delay_ms(100);								//aguarda 100ms
	Write_Data(USER_CTRL, SIG_COND_RST);		//reseta os registradores de dados do sensor
	Delay_ms(100);								//aguarda 100ms
}

//*********************************************************************
//Configuração do sensor MPU, usando interrupção.
//*********************************************************************

void MPU6500_Config(void)
{
	Delay_ms(100);		//aguarda start-up time
	MPU6500_Reset();	//reset do sensor
	//Configura��o inicial do sensor MPU-6500
	Write_Data(USER_CTRL, I2C_IF_DIS);	//desabilita a interface I2C (SPI only)
	Write_Data(PWR_MGMT_1, 0b001);		//seleciona a melhor fonte de clock dispon�vel (PLL do girosc�pio)
	Delay_ms(100);						//aguarda 100ms ap�s mudar a fonte de clock
	Write_Data(CONFIG, 0b100);			//par�metros de amostragem e filtragem (Fs=1kHz, gyro_BW=20Hz, temp_BW=20Hz)
	Write_Data(GYRO_CONFIG, 0);			//escala de leitura do girosc�pio (�250�/s)
	Write_Data(ACCEL_CONFIG, 0);		//escalada de leitura do aceler�metro (�2g)
	Write_Data(ACCEL_CONFIG2, 0b100);	//par�metros do LPF (Fs=1kHz, accel_BW=20Hz)
	Write_Data(SMPLRT_DIV, 9);			//taxa de amostragem de sa�da [1kHz/(1+9) = 100sps]

	//Habilita��o de interrup��es no pino INT do sensor
	Write_Data(INT_ENABLE, RAW_RDY_EN);	//habilita interrup��o de dados prontos
	//Write_Data(WOM_THR, 5);				//configura limiar do WakeOnMotion
	//Write_Data(ACCEL_INTEL_CTRL, (ACCEL_INTEL_EN | ACCEL_INTEL_MODE));	//habilita a inteligencia do aceler�metro
	//Write_Data(INT_ENABLE, WOM_EN);	//habilita interrup��o de WakeOnMotion
}

//*********************************************************************
//Retorna os valores de offset com o sensor estacionário
//*********************************************************************
void MPU6500_Get_Offset_Biases(void)
{
	printf("Determinando os valores de offset do sensor.\nMantenha o sensor estacion�rio e aguarde...\nEsse processo levar� em torno de 75 segundos\n\n");
	const uint16_t MAX_SAMPLE = 5000;	//quantidade de amostras usadas no c�lculo da m�dia dos valores de offset
	uint8_t rawData[14];				//array para receber os dados do sensor
	int16_t RAW_ACCEL_X, RAW_ACCEL_Y, RAW_ACCEL_Z;	//valores crus do aceler�metro
	int16_t RAW_GYRO_X, RAW_GYRO_Y, RAW_GYRO_Z;		//valores crus do girosc�pio
	int32_t ACC_RAW_ACCEL_X, ACC_RAW_ACCEL_Y, ACC_RAW_ACCEL_Z;	//valores acumulados do aceler�metro
	int32_t ACC_RAW_GYRO_X, ACC_RAW_GYRO_Y, ACC_RAW_GYRO_Z;		//valores acumulados do girosc�pio

	//acumuladores dos valores crus do sensor
	ACC_RAW_ACCEL_X = 0;
	ACC_RAW_ACCEL_Y = 0;
	ACC_RAW_ACCEL_Z = 0;
	ACC_RAW_GYRO_X = 0;
	ACC_RAW_GYRO_Y = 0;
	ACC_RAW_GYRO_Z = 0;

	uint8_t percentual = 0xFF;	//vari�vel para exibir o progresso da calibra��o
	for(uint16_t contador = 0; contador <= MAX_SAMPLE; ++contador)
	{
		Read_MData(DATA_ARRAY_POINTER, 14, rawData);	//lendo os dados do sensor

		//Separando os dados lidos do sensor
		RAW_ACCEL_X = ((uint16_t)rawData[0] << 8) | (rawData[1]);
		RAW_ACCEL_Y = ((uint16_t)rawData[2] << 8) | (rawData[3]);
		RAW_ACCEL_Z = ((uint16_t)rawData[4] << 8) | (rawData[5]);
		RAW_GYRO_X = ((uint16_t)rawData[8] << 8) | (rawData[9]);
		RAW_GYRO_Y = ((uint16_t)rawData[10] << 8) | (rawData[11]);
		RAW_GYRO_Z = ((uint16_t)rawData[12] << 8) | (rawData[13]);

		//Acumulando os dados
		ACC_RAW_ACCEL_X += RAW_ACCEL_X;
		ACC_RAW_ACCEL_Y += RAW_ACCEL_Y;
		ACC_RAW_ACCEL_Z += RAW_ACCEL_Z;
		ACC_RAW_GYRO_X += RAW_GYRO_X;
		ACC_RAW_GYRO_Y += RAW_GYRO_Y;
		ACC_RAW_GYRO_Z += RAW_GYRO_Z;

		//imprimindo o progresso da calibra��o
		if((int)floor(10*(float)contador/MAX_SAMPLE) != percentual)
		{
			percentual = (int)floor(10*(float)contador/MAX_SAMPLE);
			printf("%d0%%\n",percentual);
		}

		Delay_ms(15);
	}

	//M�dia dos valores lidos dos registradores do sensor
	RAW_ACCEL_X = ACC_RAW_ACCEL_X/MAX_SAMPLE;
	RAW_ACCEL_Y = ACC_RAW_ACCEL_Y/MAX_SAMPLE;
	RAW_ACCEL_Z = ACC_RAW_ACCEL_Z/MAX_SAMPLE;
	RAW_GYRO_X = ACC_RAW_GYRO_X/MAX_SAMPLE;
	RAW_GYRO_Y = ACC_RAW_GYRO_Y/MAX_SAMPLE;
	RAW_GYRO_Z = ACC_RAW_GYRO_Z/MAX_SAMPLE;

	printf("\nOffset m�dio dos valores lidos do sensor:\n\n");
	printf("ACCEL_X_OFFSET = %.3fg;\n", RAW_ACCEL_X*accelScalingFactor);
	printf("ACCEL_Y_OFFSET = %.3fg;\n", RAW_ACCEL_Y*accelScalingFactor);
	printf("ACCEL_Z_OFFSET = %.3fg;\n", RAW_ACCEL_Z*accelScalingFactor);
	printf("GYRO_X_OFFSET = %.3f�/s;\n", RAW_GYRO_X*gyroScalingFactor);
	printf("GYRO_Y_OFFSET = %.3f�/s;\n", RAW_GYRO_Y*gyroScalingFactor);
	printf("GYRO_Z_OFFSET = %.3f�/s;\n\n", RAW_GYRO_Z*gyroScalingFactor);

	printf("Processo conclu�do!\n");

	while(1);
}

//*********************************************************************
//Configura os valores de offset nos registradores de cancelamento de offset do sensor
//*********************************************************************
void MPU6500_Offset_Cancellation(void)
{
	//Calculando os valores de cancelamento de offset do aceler�metro
	int16_t Cancel_XA_Offset = (((((uint16_t)Read_Data(XA_OFFSET_H) << 8) | Read_Data(XA_OFFSET_L)) >> 1) + RAW_ACCEL_X_OFFSET) << 1;
	int16_t Cancel_YA_Offset = (((((uint16_t)Read_Data(YA_OFFSET_H) << 8) | Read_Data(YA_OFFSET_L)) >> 1) + RAW_ACCEL_Y_OFFSET) << 1;
	int16_t Cancel_ZA_Offset = (((((uint16_t)Read_Data(ZA_OFFSET_H) << 8) | Read_Data(ZA_OFFSET_L)) >> 1) + RAW_ACCEL_Z_OFFSET) << 1;

	//Escrevendo os valores de cancelamento de offset do aceler�metro
	Write_Data(XA_OFFSET_H, Cancel_XA_Offset >> 8);
	Write_Data(XA_OFFSET_L, Cancel_XA_Offset & 0xFF);
	Write_Data(YA_OFFSET_H, Cancel_YA_Offset >> 8);
	Write_Data(YA_OFFSET_L, Cancel_YA_Offset & 0xFF);
	Write_Data(ZA_OFFSET_H, Cancel_ZA_Offset >> 8);
	Write_Data(ZA_OFFSET_L, (Cancel_ZA_Offset & 0xFF));

	//Escrevendo os valores de cancelamento de offset do girosc�pio
	Write_Data(XG_OFFSET_H, RAW_GYRO_X_OFFSET >> 8);
	Write_Data(XG_OFFSET_L, RAW_GYRO_X_OFFSET & 0xFF);
	Write_Data(YG_OFFSET_H, RAW_GYRO_Y_OFFSET >> 8);
	Write_Data(YG_OFFSET_L, RAW_GYRO_Y_OFFSET & 0xFF);
	Write_Data(ZG_OFFSET_H, RAW_GYRO_Z_OFFSET >> 8);
	Write_Data(ZG_OFFSET_L, RAW_GYRO_Z_OFFSET & 0xFF);
}
//*********************************************************************
//Leitura de um byte
//*********************************************************************
uint8_t Read_Data(uint8_t address)
{
	GPIOD->ODR &= ~(1 << 12);			//faz o pino CS ir para n�vel baixo (inicia o comando)
	while(!(SPI2->SR & SPI_SR_TXE));	//aguarda o buffer Tx estar vazio
	SPI2->DR = (address | (1 << 7));	//envia o endere�o com o bit 7 setado
	while(!(SPI2->SR & SPI_SR_RXNE));	//aguarda o dummy byte do sensor ser recebido
	(void)SPI2->DR;						//l� o dummy byte do sensor

	SPI2->DR = 0xFF;					//envia um dummy byte para o sensor
	while(!(SPI2->SR & SPI_SR_RXNE));	//aguarda o byte de dado ser recebido
	uint8_t RX = SPI2->DR;				//l� o byte de dado do sensor
	GPIOD->ODR |= (1 << 12);			//faz o pino CS ir para n�vel alto (encerra o comando)
	Delay_us(1);						//delay entre comandos

	return RX;	//retorna o byte lido
}
//*********************************************************************
//Leitura de m�ltiplos bytes
//*********************************************************************
void Read_MData(uint8_t address, uint8_t Nbytes, uint8_t *Data)
{
	GPIOD->ODR &= ~(1 << 12);			//faz o pino CS ir para n�vel baixo (inicia o comando)
	while(!(SPI2->SR & SPI_SR_TXE));	//aguarda o buffer Tx estar vazio
	SPI2->DR = (address | (1 << 7));	//envia o endere�o com o bit 7 setado
	while(!(SPI2->SR & SPI_SR_RXNE));	//aguarda o dummy byte do sensor ser recebido
	(void)SPI2->DR;						//l� o dummy byte do sensor

	uint8_t contador = 0;
	while(Nbytes)
	{
		SPI2->DR = 0xFF;					//envia um dummy byte para o sensor
		while(!(SPI2->SR & SPI_SR_RXNE));	//aguarda o byte de dado ser recebido
		Data[contador] = SPI2->DR;			//l� o byte de dado do sensor
		++contador;
		--Nbytes;
	}

	GPIOD->ODR |= (1 << 12);	//faz o pino CS ir para n�vel alto (encerra o comando)
	Delay_us(1);				//delay entre comandos
}

//*********************************************************************
//Escrita de um byte
//*********************************************************************

void Write_Data(uint8_t address, uint8_t data)
{
	GPIOD->ODR &= ~(1 << 12);			//faz o pino CS ir para n�vel baixo (inicia o comando)
	while(!(SPI2->SR & SPI_SR_TXE));	//aguarda o buffer Tx estar vazio
	SPI2->DR = (address & ~(1<<7));		//envia o endere�o com o bit 7 resetado
	while(!(SPI2->SR & SPI_SR_RXNE));	//aguarda o dummy byte do sensor ser recebido
	(void)SPI2->DR;						//l� o dummy byte do sensor

	SPI2->DR = data;					//envia o byte de dado para o sensor
	while(!(SPI2->SR & SPI_SR_RXNE));	//aguarda o byte de dado ser recebido
	(void)SPI2->DR;						//l� o dummy byte do sensor
	GPIOD->ODR |= (1 << 12);			//faz o pino CS ir para n�vel alto (encerra o comando)
	Delay_us(1);						//delay entre comandos
}
