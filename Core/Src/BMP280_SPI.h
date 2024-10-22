/*
 * BMP280_SPI.h
 *
 *  Created on: 20 de jul de 2023
 *      Author: Fagner
 */

#ifndef BMP280_SPI_H_
#define BMP280_SPI_H_

//Definições de endereços dos registradores do sensor BMP280
#define Po			1013.25f	//pressão ao nível do mar (ajustada)
#define CALIB_REGS	0x88		//endereço inicial dos registradores de calibração
#define	RESET		0xE0		//endereço do registrador de reset
#define CTRL_MEAS	0xF4		//endereço do registrador ctrl_meas
#define CONFIG		0xF5		//endereço do registrador config
#define DATA_REGS	0xF7		//endereço de início dos dados crus


//Declarações de variáveis globais
//Parâmetros de calibração
uint16_t	BMP280_dig_T1;		//constantes de calibração da temperatura
int16_t		BMP280_dig_T2;
int16_t		BMP280_dig_T3;
uint16_t	BMP280_dig_P1;		//constantes de calibração da pressão
int16_t		BMP280_dig_P2;
int16_t		BMP280_dig_P3;
int16_t		BMP280_dig_P4;
int16_t		BMP280_dig_P5;
int16_t		BMP280_dig_P6;
int16_t		BMP280_dig_P7;
int16_t		BMP280_dig_P8;
int16_t		BMP280_dig_P9;
int32_t		t_fine;				//resolução fina da temperatura


//tipos especiais de dados
typedef int32_t		BMP280_S32_t;
typedef uint32_t	BMP280_U32_t;
typedef int64_t		BMP280_S64_t;


//Protótipos de funções de acesso ao sensor BMP280
void SPI2_Init(void);												//inicialização da SPI2 para o BMP280
void Write_Data(uint8_t address, uint8_t data);						//escrita de um byte
uint8_t Read_Data(uint8_t address);									//leitura de um byte
void Read_MData(uint8_t address, uint8_t Nbytes, uint8_t *Data);	//leitura de múltiplos bytes
void BMP280_Init(void);												//inicialização do sensor
void BMP280_Measures(float *t, float *p);							//leitura da temperatura e pressão (aritmética float)
void BMP280_Measures_int(float *t, float *p);						//leitura da temperatura e pressão (aritmética inteira)
void BMP280_Measures_double(double *t, double *p);					//leitura da temperatura e pressão (aritmética float de dupla precisão)


//Declaração de funções de acesso ao sensor BMP280
//inicialização da interface SPI2 para o sensor
void SPI2_Init(void)
{
	//Configurando os pinos da interface SPI2 em modo de função alternativa
	//PB0 como saída CS (pino CSB do sensor)
	//PB13 como SCK (pino SCL do sensor)
	//PB14 como MISO (pino SDO do sensor)
	//PB15 como MOSI (pino SDA do sensor)
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;								//habilita o clock do GPIOB
	GPIOB->MODER |= (0b10 << 30) | (0b10 << 28) | (0b10 << 26) ;		//pinos PB13, PB14 e PB15 como função alternativa
	GPIOB->AFR[1] |= (0b0101 << 28) | (0b0101 << 24) | (0b0101 << 20);	//função alternativa 5 (SPI2)
	GPIOB->ODR |= 1;		//pino inicialmente em nível alto
	GPIOB->MODER |= 0b01;	//pino como saída digital (CS)

	//Inicializando a interface SPI2 para uso com sensor BMP280
	RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;			//habilita o clock da interface SPI2
	//pino SS controlado por software, baud rate em 5,25 MHz, configura o modo master e habilita a interface
	SPI2->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI | (SPI_CR1_BR_1) | SPI_CR1_MSTR | SPI_CR1_SPE ;
}

//leitura de um byte
uint8_t Read_Data(uint8_t address)
{
	GPIOB->ODR &= ~1;						//faz o pino CS ir para nível baixo (inicia o comando)
	while(!(SPI2->SR & SPI_SR_TXE));		//aguarda o buffer Tx estar vazio
	SPI2->DR = (address | (1<<7));			//envia o endereço com o bit 7 setado
	while(!(SPI2->SR & SPI_SR_RXNE));		//aguarda o byte fictício do sensor
	(void)SPI2->DR;							//lê o byte fictício

	SPI2->DR = 0xFF;						//envia um byte fictício
	while(!(SPI2->SR & SPI_SR_RXNE));		//aguarda o byte de dado ser recebido
	uint8_t RX = SPI2->DR;					//lê o byte de dado do sensor
	GPIOB->ODR |= 1;						//faz o pino CS ir para nível alto (encerra o comando)

	return RX;								//retorna o byte lido
}

//leitura de múltiplos bytes
void Read_MData(uint8_t address, uint8_t Nbytes, uint8_t *Data)
{
	GPIOB->ODR &= ~1;						//faz o pino CS ir para nível baixo (inicia o comando)
	while(!(SPI2->SR & SPI_SR_TXE));		//aguarda o buffer Tx estar vazio
	SPI2->DR = (address | (1<<7));			//envia o endereço com o bit 7 setado
	while(!(SPI2->SR & SPI_SR_RXNE));		//aguarda o byte fictício do sensor
	(void)SPI2->DR;							//lê o byte fictício

	uint8_t contador = 0;
	while(Nbytes)
	{
		SPI2->DR = 0xFF;					//envia um byte fictício
		while(!(SPI2->SR & SPI_SR_RXNE));	//aguarda o byte de dado ser recebido
		Data[contador] = SPI2->DR;			//lê o byte de dado do sensor
		++contador;
		--Nbytes;
	}

	GPIOB->ODR |= 1;						//faz o pino CS ir para nível alto (encerra o comando)
}

//escrita de um byte
void Write_Data(uint8_t address, uint8_t data)
{
	GPIOB->ODR &= ~1;						//faz o pino CS ir para nível baixo (inicia o comando)
	while(!(SPI2->SR & SPI_SR_TXE));		//aguarda o buffer Tx estar vazio
	SPI2->DR = (address & ~(1<<7));			//envia o endereço com o bit 7 resetado
	while(!(SPI2->SR & SPI_SR_RXNE));		//aguarda o byte fictício do sensor ser recebido
	(void)SPI2->DR;							//lê o byte fictício

	SPI2->DR = data;						//envia o byte de dado para o sensor
	while(!(SPI2->SR & SPI_SR_RXNE));		//aguarda o byte fictício ser recebido
	(void)SPI2->DR;							//lê o byte fictício
	GPIOB->ODR |= 1;						//faz o pino CS ir para nível alto (encerra o comando)
}

//inicialização do sensor
void BMP280_Init(void)
{
	HAL_Delay(5);				//aguarda o start-up time (mínimo de 2ms)

	//Reseta o sensor e aguarda o start-up time novamente (mínimo de 2ms)
	Write_Data(RESET, 0xB6);	//escrita da constante 0xB6 no registrador de reset
	HAL_Delay(5);				//aguarda 5 ms

	//Coletando os parâmetros de calibração
	uint8_t param[24];
	Read_MData(CALIB_REGS, 24, param);

	//Extraindo os dados de calibração da temperatura
	BMP280_dig_T1 = (param[1] << 8) | param[0];
	BMP280_dig_T2 = (param[3] << 8) | param[2];
	BMP280_dig_T3 = (param[5] << 8) | param[4];

	//Extraindo os dados de calibração da pressão
	BMP280_dig_P1 = (param[7] << 8) | param[6];
	BMP280_dig_P2 = (param[9] << 8) | param[8];
	BMP280_dig_P3 = (param[11] << 8) | param[10];
	BMP280_dig_P4 = (param[13] << 8) | param[12];
	BMP280_dig_P5 = (param[15] << 8) | param[14];
	BMP280_dig_P6 = (param[17] << 8) | param[16];
	BMP280_dig_P7 = (param[19] << 8) | param[18];
	BMP280_dig_P8 = (param[21] << 8) | param[20];
	BMP280_dig_P9 = (param[23] << 8) | param[22];

	//filtro IIR (coeficiente = 16)
	Write_Data(CONFIG, 0x1C);

	//Saída do sensor do modo sleep (operação em modo normal)
	//20 bits de resolução, oversampling x16 na pressão, resolução de 0.16 Pa
	//20 bits de resolução, oversampling x16 na temperatura, resolução de 0.0003 °C
	Write_Data(CTRL_MEAS, 0xFF);
}

//leitura da temperatura e pressão com aritmética float
void BMP280_Measures(float *t, float *p)
{
	//Lendo os registradores com os valores crus das grandezas
	uint8_t RawData[6];
	Read_MData(DATA_REGS, 6, RawData);	//burst read

	//Extraindo os dados crus da pressão e temperatura
	int32_t adc_P = ((RawData[0] << 16) | (RawData[1] << 8) | RawData[2]) >> 4;	//pressão raw
	int32_t adc_T = ((RawData[3] << 16 )| (RawData[4] << 8 )| RawData[5]) >> 4;	//temperatura raw

	//calculando a temperatura
	float var1, var2, T;
	var1 = (((float)adc_T)/16384 - ((float)BMP280_dig_T1)/1024) * ((float)BMP280_dig_T2);
	var2 = ((((float)adc_T)/131072 - ((float)BMP280_dig_T1)/8192) *
		   (((float)adc_T)/131072 - ((float)BMP280_dig_T1)/8192)) * ((float)BMP280_dig_T3);
	t_fine = (var1 + var2);
	T = (var1 + var2)/5120;
	*t = T;		//retorna T em °C (retorno em float)

	//calculando a pressão
	float P;
	var1 = ((float)t_fine/2) - 64000;
	var2 = var1 * var1 * ((float)BMP280_dig_P6)/32768;
	var2 = var2 + var1 * ((float)BMP280_dig_P5) * 2;
	var2 = (var2/4)+(((float)BMP280_dig_P4) * 65536);
	var1 = (((float)BMP280_dig_P3) * var1 * var1/524288 + ((float)BMP280_dig_P2) * var1)/524288;
	var1 = (1 + var1/32768)*((float)BMP280_dig_P1);
	if (var1 == 0) P = 0; //evita exceção causada por divisão por zero
	else
	{
		P = 1048576 - (float)adc_P;
		P = (P - (var2/4096)) * 6250/var1;
		var1 = ((float)BMP280_dig_P9) * P * P/2147483648;
		var2 = P * ((float)BMP280_dig_P8)/32768;
		P = P + (var1 + var2 + ((float)BMP280_dig_P7))/16;
		*p = P/100;	//retorna P em hPa (retorno em float)
	}
}

//leitura da temperatura e pressão com aritmética inteira
void BMP280_Measures_int(float *t, float *p)
{
	//Lendo os registradores com os valores crus das grandezas
	uint8_t RawData[6];
	Read_MData(DATA_REGS, 6, RawData);	//burst read

	//Extraindo os dados crus da pressão e temperatura
	int32_t adc_P = ((RawData[0] << 16) | (RawData[1] << 8) | RawData[2]) >> 4;	//pressão raw
	int32_t adc_T = ((RawData[3] << 16 )| (RawData[4] << 8 )| RawData[5]) >> 4;	//temperatura raw

	//calculando a temperatura
	BMP280_S32_t var1, var2, T;
	var1 = ((((adc_T >> 3) - ((BMP280_S32_t)BMP280_dig_T1 << 1))) * ((BMP280_S32_t)BMP280_dig_T2)) >> 11;
	var2 = (((((adc_T >> 4) - ((BMP280_S32_t)BMP280_dig_T1)) * ((adc_T >> 4) - ((BMP280_S32_t)BMP280_dig_T1))) >> 12) * ((BMP280_S32_t)BMP280_dig_T3)) >> 14;

	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	*t = (float)T/100;		//retorna T em °C (retorno em float)

	//calculando a pressão
	BMP280_S64_t var1_, var2_, P;
	var1_ = ((BMP280_S64_t)t_fine) - 128000;
	var2_ = var1_ * var1_ * (BMP280_S64_t)BMP280_dig_P6;
	var2_ = var2_ + ((var1_ * (BMP280_S64_t)BMP280_dig_P5) << 17);
	var2_ = var2_ + (((BMP280_S64_t)BMP280_dig_P4) << 35);
	var1_ = ((var1_ * var1_ * (BMP280_S64_t)BMP280_dig_P3) >> 8) + ((var1_ * (BMP280_S64_t)BMP280_dig_P2) << 12);
	var1_ = (((((BMP280_S64_t)1) << 47) + var1_)) * ((BMP280_S64_t)BMP280_dig_P1) >> 33;
	if (var1_ == 0)
	{
		P = 0; //evita exceção causada por divisão por zero
	}
	else
	{
		P = 1048576 - adc_P;
		P = (((P << 31) - var2_) * 3125)/var1_;
		var1_ = (((BMP280_S64_t)BMP280_dig_P9) * (P >> 13) * (P >> 13)) >> 25;
		var2_ = (((BMP280_S64_t)BMP280_dig_P8) * P) >> 19;
		P = ((P + var1_ + var2_) >> 8) + (((BMP280_S64_t)BMP280_dig_P7) << 4);
	}
	*p = (float)((BMP280_U32_t)P)/(25600);	//retorna P em hPa (retorno em float)
}

//leitura da temperatura e pressão com aritmética double
void BMP280_Measures_double(double *t, double *p)
{
	//Lendo os registradores com os valores crus das grandezas
	uint8_t RawData[6];
	Read_MData(DATA_REGS, 6, RawData);	//burst read

	//Extraindo os dados crus da pressão e temperatura
	int32_t adc_P = ((RawData[0] << 16) | (RawData[1] << 8) | RawData[2]) >> 4;	//pressão raw
	int32_t adc_T = ((RawData[3] << 16 )| (RawData[4] << 8 )| RawData[5]) >> 4;	//temperatura raw

	//calculando a temperatura
	double var1, var2, T;
	var1 = (((double)adc_T)/16384.0 - ((double)BMP280_dig_T1)/1024.0) * ((double)BMP280_dig_T2);
	var2 = ((((double)adc_T)/131072.0 - ((double)BMP280_dig_T1)/8192.0) *
	(((double)adc_T)/131072.0 - ((double)BMP280_dig_T1)/8192.0)) * ((double)BMP280_dig_T3);
	t_fine = (BMP280_S32_t)(var1 + var2);
	T = (var1 + var2) / 5120.0;
	*t = T;		//retorna T em °C (retorno em double)

	//calculando a pressão
	double var1_, var2_, P;
	var1_ = ((double)t_fine/2.0) - 64000.0;
	var2_ = var1_ * var1_ * ((double)BMP280_dig_P6) / 32768.0;
	var2_ = var2_ + var1_ * ((double)BMP280_dig_P5) * 2.0;
	var2_ = (var2_/4.0)+(((double)BMP280_dig_P4) * 65536.0);
	var1_ = (((double)BMP280_dig_P3) * var1_ * var1_ / 524288.0 + ((double)BMP280_dig_P2) * var1_) / 524288.0;
	var1_ = (1.0 + var1_ / 32768.0)*((double)BMP280_dig_P1);
	if (var1_ == 0.0)
	{
		P = 0.0; //evita exceção causada por divisão por zero
	}
	else
	{
		P = 1048576.0 - (double)adc_P;
		P = (P - (var2_ / 4096.0)) * 6250.0 / var1_;
		var1_ = ((double)BMP280_dig_P9) * P * P / 2147483648.0;
		var2_ = P * ((double)BMP280_dig_P8) / 32768.0;
		P = P + (var1_ + var2_ + ((double)BMP280_dig_P7)) / 16.0;
		*p = P/100;	//retorna P em hPa (retorno em double)
	}
}

#endif /* BMP280_SPI_H_ */
