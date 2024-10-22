/*
 * DEFINES.h
 *
 *  Created on: Set 19, 2024
 *      Author: Gabriel
 */

#ifndef DEFINES_H_
#define DEFINES_H_


//Definições globais do SAF-T
#define	TRUE		1
#define	FALSE		0

//Variáveis globais
const uint8_t Tx_Data[15] = {(0x3B | (1<<7)), 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};	//array para envio dos dados para o sensor

const float b[5] = {0.43284664, -1.73138658, 2.59707987, -1.73138658, 0.43284664}; 		//coeficientes das amostras no filtro passa alta de quinta ordem de 10Hz
const float a[5] = {1.0, -2.36951301, 2.31398841, -1.05466541, 0.18737949}; 			//coeficientes das saídas no filtro passa alta de quinta ordem de 10Hz

//Arrays para armazenar as amostras e saídas para o filtro passa alta
float amostras_x[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
float saidas_x[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
float amostras_y[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
float saidas_y[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
float amostras_z[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
float saidas_z[5] = {0.0, 0.0, 0.0, 0.0, 0.0};

//Parametros utilizados na fusão dos filtros com o filtro de Kalman
float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;
float AngleRoll, AnglePitch;
float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2*2;
float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch = 2*2;
float Kalman1DOutput[] = {0,0};
uint32_t Address = 0x080E0000;


uint8_t Rx_Data[15];											//array para recebimento dos dados do sensor
uint8_t Sensor_Data_Ready = FALSE;								//flag que indica o estado dos dados do sensor após o DMA
int16_t RAW_ACCEL_X, RAW_ACCEL_Y, RAW_ACCEL_Z;					//valores crus do acelerômetro
int16_t RAW_GYRO_X, RAW_GYRO_Y, RAW_GYRO_Z;						//valores crus do giroscópio
int16_t RAW_TEMP;												//valor cru da temperatura
float ACCEL_X, ACCEL_Y, ACCEL_Z, GYRO_X, GYRO_Y, GYRO_Z, TEMP;	//valores dos sensores já escalados
float ACCEL_FILTERED_X, ACCEL_FILTERED_Y, ACCEL_FILTERED_Z;		//variáveis para guardar os valores filtrados da aceleração com filtro passa alta

//Funções
float passa_alta_butterworth(float new_input, float *x, float *y); //Filtro passa alta de quinta ordem
void FLASH_ClearFlag(uint32_t FLASH_FLAG);


void DMA1_Config(void)
{
	//Configuração do canal 0 do stream 4 do DMA1 (SPI2_TX)
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;			//habilita o clock da interface do DMA1

	DMA1_Stream4->PAR = (uint32_t) &(SPI2->DR);	//ponteiro do periférico para onde os dados devem ser transferidos
	DMA1_Stream4->CR &= ~DMA_SxCR_CHSEL;		//seleciona canal 0
	DMA1_Stream4->NDTR = 15;					//quantidade de dados a serem transferidos para o periférico
	DMA1_Stream4->CR |= DMA_SxCR_DIR_0;			//direção da memória para o periférico
	DMA1_Stream4->CR |= DMA_SxCR_MINC;			//ponteiro da memória pós incrementado automaticamente
	DMA1_Stream4->CR &= ~DMA_SxCR_PINC;			//ponteiro do periférico fixo
	DMA1_Stream4->CR &= ~DMA_SxCR_PSIZE;		//tamanho do dado do periférico (8 bits)
	DMA1_Stream4->CR &= ~DMA_SxCR_MSIZE;		//tamanho do dado na memória (8 bits)
	DMA1_Stream4->FCR &= ~DMA_SxFCR_DMDIS;		//habilita o modo de transferência direta (sem FIFO)

	//Configuração do canal 0 do stream 3 do DMA1 (SPI2_RX)
	DMA1_Stream3->PAR = (uint32_t) &(SPI2->DR);	//ponteiro do periférico de onde os dados devem ser transferidos
	DMA1_Stream3->CR &= ~DMA_SxCR_CHSEL;		//seleciona canal 0
	DMA1_Stream3->NDTR = 15;					//quantidade de dados a serem transferidos para a memória
	DMA1_Stream3->CR &= ~DMA_SxCR_DIR;			//direção do periférico para a memória
	DMA1_Stream3->CR |= DMA_SxCR_MINC;			//ponteiro da memória pós incrementado automaticamente
	DMA1_Stream3->CR &= ~DMA_SxCR_PINC;			//ponteiro do periférico fixo
	DMA1_Stream3->CR &= ~DMA_SxCR_PSIZE;		//tamanho do dado do periférico (8 bits)
	DMA1_Stream3->CR &= ~DMA_SxCR_MSIZE;		//tamanho do dado na memória (8 bits)
	DMA1_Stream3->FCR &= ~DMA_SxFCR_DMDIS;		//habilita o modo de transferência direta (sem FIFO)

	DMA1_Stream3->CR |= DMA_SxCR_TCIE;			//habilita a interrupção de transferência completa do Stream3 do DMA1
	//NVIC_EnableIRQ(DMA1_Stream3_IRQn);			//habilita a interrupção no NVIC
	HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
}

void PB9_Int_Config(void)
{
	//Configuração do pino PB9 como entrada digital com resistor de pull-down
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;	//habilita o clock do GPIOB
	GPIOB->MODER &= ~(0b11 << 18);			//seleciona modo de entrada digital no pino PB9
	GPIOB->PUPDR |= 0b10 << 18;				//habilita o resistor de pull-down no pino PB9

	//Configuração da EXTI9 para receber pulsos externos em PB9
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;	//habilita o clock de SYSCFG
	SYSCFG->EXTICR[2] |= 0b0001 << 4;		//seleciona PB9 como gatilho de EXTI9
	EXTI->RTSR |= 1 << 9;			 		//habilita a detecção por borda de subida em PB9
	EXTI->IMR |= 1 << 9;				 	//habilita a interrupção EXTI9 no controlador EXTI
	NVIC_EnableIRQ(EXTI9_5_IRQn);			//habilita a interrupção EXTI9 no NVIC
}

float passa_alta_butterworth(float new_input, float *x, float *y) {

//    //atualizando as amostras de entrada
    for (int i = 0; i < 4; i++) {			//Atualizar a lista de entradas
           x[4-i] = x[3-i];
       }

    x[0] = new_input;						//Ultima amostra obtida

    y[0] = b[0]*x[0] + b[1]*x[1] + b[2]*x[2] + b[3]*x[3] + b[4]*x[4]

    		 - a[1]*y[1] - a[2]*y[2] - a[3]*y[3] - a[4]*y[4];

    //    //atualizando as amostras de saída
    for (int i = 0; i < 4; i++) {			// Atualizar a lista de saídas
        y[4-i] = y[3-i];
    }

    return y[0];
}

void kalman_1d(float *KalmanState, float *KalmanUncertainty, float *KalmanInput, float *KalmanMeasurement) {

  *KalmanState = *KalmanState + 0.004 * *KalmanInput;
  *KalmanUncertainty = *KalmanUncertainty + 0.004 * 0.004 * 4.0 * 4.0;

  float KalmanGain = *KalmanUncertainty * 1.0/(1.0**KalmanUncertainty + 3.0 * 3.0);

  *KalmanState = *KalmanState + KalmanGain * (*KalmanMeasurement - *KalmanState);
  *KalmanUncertainty = (1.0 - KalmanGain) * *KalmanUncertainty;
  Kalman1DOutput[0] = *KalmanState;
  Kalman1DOutput[1] = *KalmanUncertainty;
}

void gyro_signals(void) {
	RAW_ACCEL_X = (((uint16_t) Rx_Data[1] << 8) | (Rx_Data[2]));
	RAW_ACCEL_Y = (((uint16_t) Rx_Data[3] << 8) | (Rx_Data[4]));
	RAW_ACCEL_Z = (((uint16_t) Rx_Data[5] << 8) | (Rx_Data[6]));
	RAW_GYRO_X = (((int16_t) Rx_Data[9] << 8) | (Rx_Data[10]));
	RAW_GYRO_Y = (((int16_t) Rx_Data[11] << 8) | (Rx_Data[12]));
	RAW_GYRO_Z = (((int16_t) Rx_Data[13] << 8) | (Rx_Data[14]));

	RateRoll = (float) RAW_GYRO_X / 131.0;
	RatePitch = (float) RAW_GYRO_Y / 131.0;
	RateYaw = (float) RAW_GYRO_Z / 131.0;
	ACCEL_X = (float) RAW_ACCEL_X / 16384.0;
	ACCEL_Y = (float) RAW_ACCEL_Y / 16384.0;
	ACCEL_Z = (float) RAW_ACCEL_Z / 16384.0;
	AngleRoll = atan(ACCEL_Y / sqrt(ACCEL_X * ACCEL_X + ACCEL_Z * ACCEL_Z)) * 1 / (3.142/180.0);
	AnglePitch = -atan(ACCEL_X / sqrt(ACCEL_Y * ACCEL_Y + ACCEL_Z * ACCEL_Z)) * 1 /(3.142/180.0);
}

void calibration_DMT(void){


		printf(
			"Determinando os valores de offset do sensor.\nMantenha o sensor estacion�rio e aguarde...\nEsse processo levar� em torno de 75 segundos\n\n");
	const uint16_t MAX_SAMPLE = 5000;//quantidade de amostras usadas no c�lculo da m�dia dos valores de offset
	uint8_t rawData[14];			//array para receber os dados do sensor
	int16_t RAW_ACCEL_X, RAW_ACCEL_Y, RAW_ACCEL_Z;//valores crus do aceler�metro
	int16_t RAW_GYRO_X, RAW_GYRO_Y, RAW_GYRO_Z;	//valores crus do girosc�pio
	int32_t ACC_RAW_ACCEL_X, ACC_RAW_ACCEL_Y, ACC_RAW_ACCEL_Z;//valores acumulados do aceler�metro
	int32_t ACC_RAW_GYRO_X, ACC_RAW_GYRO_Y, ACC_RAW_GYRO_Z;	//valores acumulados do girosc�pio

	//acumuladores dos valores crus do sensor
	ACC_RAW_ACCEL_X = 0;
	ACC_RAW_ACCEL_Y = 0;
	ACC_RAW_ACCEL_Z = 0;
	ACC_RAW_GYRO_X = 0;
	ACC_RAW_GYRO_Y = 0;
	ACC_RAW_GYRO_Z = 0;

	uint8_t percentual = 0xFF;	//vari�vel para exibir o progresso da calibra��o
	for (uint16_t contador = 0; contador <= MAX_SAMPLE; ++contador) {
		Read_MData(DATA_ARRAY_POINTER, 14, rawData);//lendo os dados do sensor

		//Separando os dados lidos do sensor
		RAW_ACCEL_X = ((uint16_t) rawData[0] << 8) | (rawData[1]);
		RAW_ACCEL_Y = ((uint16_t) rawData[2] << 8) | (rawData[3]);
		RAW_ACCEL_Z = ((uint16_t) rawData[4] << 8) | (rawData[5]);
		RAW_GYRO_X = ((uint16_t) rawData[8] << 8) | (rawData[9]);
		RAW_GYRO_Y = ((uint16_t) rawData[10] << 8) | (rawData[11]);
		RAW_GYRO_Z = ((uint16_t) rawData[12] << 8) | (rawData[13]);

		//Acumulando os dados
		ACC_RAW_ACCEL_X += RAW_ACCEL_X;
		ACC_RAW_ACCEL_Y += RAW_ACCEL_Y;
		ACC_RAW_ACCEL_Z += RAW_ACCEL_Z;
		ACC_RAW_GYRO_X += RAW_GYRO_X;
		ACC_RAW_GYRO_Y += RAW_GYRO_Y;
		ACC_RAW_GYRO_Z += RAW_GYRO_Z;

		//imprimindo o progresso da calibra��o
		if ((int) floor(10 * (float) contador / MAX_SAMPLE) != percentual) {
			percentual = (int) floor(10 * (float) contador / MAX_SAMPLE);
			printf("%d0%%\n", percentual);
		}

		Delay_ms(15);
	}

	//M�dia dos valores lidos dos registradores do sensor
	RAW_ACCEL_X = (ACC_RAW_ACCEL_X / MAX_SAMPLE);
	RAW_ACCEL_Y = (ACC_RAW_ACCEL_Y / MAX_SAMPLE);
	RAW_ACCEL_Z = (ACC_RAW_ACCEL_Z / MAX_SAMPLE);
	RAW_GYRO_X = (ACC_RAW_GYRO_X / MAX_SAMPLE);
	RAW_GYRO_Y = (ACC_RAW_GYRO_Y / MAX_SAMPLE);
	RAW_GYRO_Z = (ACC_RAW_GYRO_Z / MAX_SAMPLE);

	printf("\nOffset m�dio dos valores lidos do sensor:\n\n");
	printf("ACCEL_X_OFFSET = %.3fg;\n", RAW_ACCEL_X * accelScalingFactor);
	printf("ACCEL_Y_OFFSET = %.3fg;\n", RAW_ACCEL_Y * accelScalingFactor);
	printf("ACCEL_Z_OFFSET = %.3fg;\n", RAW_ACCEL_Z * accelScalingFactor);
	printf("GYRO_X_OFFSET = %.3f�/s;\n", RAW_GYRO_X * gyroScalingFactor);
	printf("GYRO_Y_OFFSET = %.3f�/s;\n", RAW_GYRO_Y * gyroScalingFactor);
	printf("GYRO_Z_OFFSET = %.3f�/s;\n\n\n\n\n", RAW_GYRO_Z * gyroScalingFactor);

	//Valores inteiros dos offsets mensurados
	RAW_ACCEL_X = round(-(RAW_ACCEL_X * accelScalingFactor) * 1024.0);
	RAW_ACCEL_Y = round(-(RAW_ACCEL_Y * accelScalingFactor) * 1024.0);
	RAW_ACCEL_Z = round(-(1 - (RAW_ACCEL_Z * accelScalingFactor)) * 1024.0);
	RAW_GYRO_X = round(-(RAW_GYRO_X * gyroScalingFactor) * 32.768);
	RAW_GYRO_Y = round(-(RAW_GYRO_Y * gyroScalingFactor) * 32.768);
	RAW_GYRO_Z = round(-(RAW_GYRO_Z * gyroScalingFactor) * 32.768);

	int16_t Cancel_XA_Offset = (((((uint16_t) Read_Data(XA_OFFSET_H) << 8)
			| Read_Data(XA_OFFSET_L)) >> 1) + RAW_ACCEL_X) << 1;
	int16_t Cancel_YA_Offset = (((((uint16_t) Read_Data(YA_OFFSET_H) << 8)
			| Read_Data(YA_OFFSET_L)) >> 1) + RAW_ACCEL_Y) << 1;
	int16_t Cancel_ZA_Offset = (((((uint16_t) Read_Data(ZA_OFFSET_H) << 8)
			| Read_Data(ZA_OFFSET_L)) >> 1) + RAW_ACCEL_Z) << 1;

	int32_t RAW_GYRO_X_32 = (int32_t) RAW_GYRO_X;
	int32_t RAW_GYRO_Y_32 = (int32_t) RAW_GYRO_Y;
	int32_t RAW_GYRO_Z_32 = (int32_t) RAW_GYRO_Z;
	int32_t Cancel_XA_Offset_32 = (int32_t) Cancel_XA_Offset;
	int32_t Cancel_YA_Offset_32 = (int32_t) Cancel_YA_Offset;
	int32_t Cancel_ZA_Offset_32 = (int32_t) Cancel_ZA_Offset;


	printf("\nValor inteiro cru dos offsets obtidos:\n\n");
	printf("ACCEL_X_OFFSET = %ld;\n", Cancel_XA_Offset_32);
	printf("ACCEL_Y_OFFSET = %ld;\n", Cancel_YA_Offset_32);
	printf("ACCEL_Z_OFFSET = %ld;\n", Cancel_ZA_Offset_32);
	printf("GYRO_X_OFFSET = %ld;\n", RAW_GYRO_X_32);
	printf("GYRO_Y_OFFSET = %ld;\n", RAW_GYRO_Y_32);
	printf("GYRO_Z_OFFSET = %ld.\n", RAW_GYRO_Z_32);

	// Salvar os dados coletados pelos offsets na memória flash
	HAL_FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR );
	FLASH_Erase_Sector(FLASH_SECTOR_11, FLASH_VOLTAGE_RANGE_3);

	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, Cancel_XA_Offset_32);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address + 4, Cancel_YA_Offset_32);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address + 8, Cancel_ZA_Offset_32);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address + 12, RAW_GYRO_X_32);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address + 16, RAW_GYRO_Y_32);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address + 20, RAW_GYRO_Z_32);

	HAL_FLASH_Lock();


	printf("O DMT já foi calibrado!");


//	while(1);

}

void leitura_flash(void){

	int32_t read_Data [6];

	for (int i = 0; i < 6; i++) {
		read_Data[i] = * (__IO uint32_t*) Address;
		Address += 4;
	}

	int16_t Cancel_XA_Offset = (int16_t) read_Data[0];
	int16_t Cancel_YA_Offset = (int16_t) read_Data[1];
	int16_t Cancel_ZA_Offset = (int16_t) read_Data[2];
	int16_t GYRO_X_OFFSET = (int16_t) read_Data[3];
	int16_t GYRO_Y_OFFSET = (int16_t) read_Data[4];
	int16_t GYRO_Z_OFFSET = (int16_t) read_Data[5];

	//Escrevendo os valores de cancelamento de offset do aceler�metro
	Write_Data(XA_OFFSET_H, Cancel_XA_Offset >> 8);
	Write_Data(XA_OFFSET_L, Cancel_XA_Offset & 0xFF);
	Write_Data(YA_OFFSET_H, Cancel_YA_Offset >> 8);
	Write_Data(YA_OFFSET_L, Cancel_YA_Offset & 0xFF);
	Write_Data(ZA_OFFSET_H, Cancel_ZA_Offset >> 8);
	Write_Data(ZA_OFFSET_L, (Cancel_ZA_Offset & 0xFF));

	//Escrevendo os valores de cancelamento de offset do girosc�pio
	Write_Data(XG_OFFSET_H, GYRO_X_OFFSET >> 8);
	Write_Data(XG_OFFSET_L, GYRO_X_OFFSET & 0xFF);
	Write_Data(YG_OFFSET_H, GYRO_Y_OFFSET >> 8);
	Write_Data(YG_OFFSET_L, GYRO_Y_OFFSET & 0xFF);
	Write_Data(ZG_OFFSET_H, GYRO_Z_OFFSET >> 8);
	Write_Data(ZG_OFFSET_L, GYRO_Z_OFFSET & 0xFF);

}

void FLASH_ClearFlag(uint32_t FLASH_FLAG)
{
  /* Check the parameters */
  assert_param(IS_FLASH_CLEAR_FLAG(FLASH_FLAG));

  /* Clear the flags */
  FLASH->SR = FLASH_FLAG;
}

#endif /* DEFINES_H_ */
