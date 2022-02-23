/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "utils.h"
#include "usbd_cdc_if.h"
#include "string.h"
#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/************************* Valores MAX-MIN de motores *****************************/
#define PWMAMAX			6500
#define PWMAMIN			3000

//#define PWMBMAX			4800
#define PWMBMIN			6200
/**********************************************************************************/

/************************* Longitud de respuestas ESP *****************************/
//#define LONG_CWJAP_	35 //MICRO
//#define LONG_ANS_CWJAP_ 87 //MICRO
#define LONG_CWJAP_	45 //CASA
#define LONG_ANS_CIPSTART 59 //CASA
#define LONG_ANS_CWJAP_ 97 //CASA
//#define LONG_CWJAP_	47 //FACU
//#define LONG_ANS_CWJAP_ 93 //FACU
//#define LONG_ANS_CIPSTART 62 //FACU
/**********************************************************************************/

/************************************ Bits ****************************************/
#define espConnected 			flag1.bit.b0
#define PCConnected				flag1.bit.b1
#define SendAlive				flag1.bit.b2
#define DecodeHeaderESP			flag1.bit.b3
#define RespMotor				flag1.bit.b4
#define ESPReadyToRecyb			flag1.bit.b5
#define DecodeIPD				flag1.bit.b6
#define ECOCIPSEND0xF0			flag1.bit.b7

#define ResetESP				flag2.bit.b0
#define SentDataESP				flag2.bit.b1
#define SendADC					flag2.bit.b2
#define Start					flag2.bit.b3

/**********************************************************************************/
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/******************************** Comandos AT ***********************************/
const char AT[]="AT\r\n";
//const char CWJAP_[]="AT+CWJAP=\"MICROS\",\"micros1234567\"\r\n";
const char CWJAP_[]="AT+CWJAP=\"InternetPlus_861f50\",\"wlan79e0af\"\r\n";//45
//const char CWJAP_[]="AT+CWJAP=\"FCAL\",\"fcalconcordia.06-2019\"\r\n";
const char CIFSR[] = "AT+CIFSR\r\n";
const char CIPMUX[] = "AT+CIPMUX=0\r\n";
const char CIPSTART[]="AT+CIPSTART=\"UDP\",\"192.168.1.6\",30011,3011\r\n";//44
//const char CIPSTART[]="AT+CIPSTART=\"UDP\",\"172.23.247.155\",30011,3011\r\n";//47
const char CIPSEND[] = "AT+CIPSEND=";
const char CIPCLOSE[] = "AT+CIPCLOSE\r\n";
const char CWQAP[] = "AT+CWQAP\r\n";
const char CWMODE[] = "AT+CWMODE=3\r\n";
/********************************************************************************/

/*************************** Respuestas comandos AT *****************************/
const char ANS_CWMODE[] = "AT+CWMODE=3\r\n\r\nOK\r\n"; //19
const char ANS_CWQAP[] = "AT+CWQAP\r\n\r\nOK\r\n"; //16
//const char ANS_CWJAP_[]="AT+CWJAP=\"MICROS\",\"micros1234567\"\r\nWIFI CONNECTED\r\nWIFI GOT IP\r\n\r\nOK\r\n";//70
//Longitud de ANS_CWJAP_[] = 68+LongSSID+Contra//
const char ANS_CWJAP_[]="AT+CWJAP=\"InternetPlus_861f50\",\"wlan79e0af\"\r\nWIFI DISCONNECT\r\nWIFI CONNECTED\r\nWIFI GOT IP\r\n\r\nOK\r\n";//97
//const char ANS_CWJAP_[]="AT+CWJAP=\"FCAL\",\"fcalconcordia.06-2019\"\r\nWIFI DISCONNECT\r\nWIFI CONNECTED\r\nWIFI GOT IP\r\n\r\nOK\r\n";//93
const char ANS_CIPMUX[] = "AT+CIPMUX=0\r\n\r\nOK\r\n"; //19
//const char ANS_CIFSR[] = "AT+CIFSR\r\n";
//const char ANS_CIFSR_STAIP[] = "+CIFSR:STAIP,\"";
//const char ANS_CIPSTART[]="AT+CIPSTART=\"UDP\",\"172.23.247.155\",30011,3011\r\nCONNECT\r\n\r\nOK\r\n";//62
const char ANS_CIPSTART[]="AT+CIPSTART=\"UDP\",\"192.168.1.6\",30011,3011\r\nCONNECT\r\n\r\nOK\r\n";//59
const char ANS_AT[] = "AT\r\n\r\nOK\r\n";
//const char AUTOMATIC_WIFI_CONNECTED[] = {"rIFI CONNECTED\r\nWIFI GOT IP\r\n"};
const char WIFI_DISCONNECT[] = "WIFI DISCONNECT\r\n";
//const char CIFSR_STAIP[] = "+CIFSR:STAIP,\"";
//const char OK[]="\r\nOK\r\n";
//const char CIPSEND1[]={"AT+CIPSEND="};
//const char CIPSEND2[]={"\r\n\r\nOK\r\n>"};
//const char CIPSEND3[]={"\r\nRecv "};
//const char CIPSEND4[]={" bytes\r\n\r\nSEND OK\r\n"};
const char IPD[]={"\r\n+IPD,"};
const char Error[] = "\r\nERROR\r\n";
/*********************************************************************************/

/********************* Cálculo error algoritmo cuadratico ************************/
const int COORD_SENSORES[]={-45,-35,-25,-15,-5,5,15,25,35,45};
static uint8_t FirtScan;
uint16_t bufADC[32][8];
volatile uint8_t iAdc;
uint8_t posMINCenter, posMINDerecha, posMINIzquierda;
int16_t sensorValue,x2_x1,x2_x3,x2_x1cuad,x2_x3cuad;
int32_t fx2_fx3,fx2_fx1,denominador;
//int32_t error;
/*********************************************************************************/
/************************************* PID y PWM***************************************/
int16_t IntegralMA, DerivativoMA, ProporcionalMB, IntegralMB, DerivativoMB, lastError;//, M1Power, M2Power;
uint8_t KpMA, KiMA, KdMA, KpMB, KiMB, KdMB;
//volatile uint8_t TimeOutPID;
//uint16_t PWMA, PWMB;
/*********************************************************************************/

/***************************** Contadores timmer *********************************/
volatile uint16_t DecodeTimeOut, On5ms, On100ms, On200ms, On3000ms, On500ms, Count100ms, Countms, Count5ms, Count500ms;
/*********************************************************************************/

/********************** Comunicación - Buffer e indices  *************************/
volatile _Rx RXUSB, RXUSART1;
_Tx TXUSB, TXUSART1;
uint8_t rxUSBBuff[256], rxUSART1Buff[256] = {};
uint8_t txUSBBuff[256], txUSART1Buff[256];
uint8_t DecodeCIPSEND, IndiceIPD, IndiceDisconnect, IndiceError;
uint8_t Estado, Indice, Data;
/*********************************************************************************/
_work w, M1Power, M2Power, error, PWMA, PWMB, PWM_A, PWM_B, PWMBaseA, PWMBaseB, ProporcionalMA;//, Kp, Ki, Kd;

volatile _sFlag flag1, flag2;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/**********************************************************************************/
/*************************** Prototipos de Funciones ******************************/
/**********************************************************************************/

/*
 * Captura datos entrantes por USB
 * Incrementa indice de buffer de recepción
 *
 */
void MyCallBackOnUSBData(uint8_t *buf, uint32_t len);
/*
 * Decodifica protocolo de comunicacion
 * Calcula checksum
 * Detecta si hay comando
 *
 */
void DecodeHeader(_Rx *RX);
/*
 * Decodifica comando
 * Envida datos segun comando
 *
 */
void DecodeCmd(_Rx *RX, _Tx *TX);
/*
 * Carga buffer Tx en el buffer de USB o USART
 *
 */
void PutBuffOnTx(_Tx *TX, uint8_t *buf, uint8_t length);
/*
 * Carga byte Tx en el buffer de USB o USART
 *
 */
void PutByteOnTx(_Tx *TX, uint8_t value);
/*
 * Carga de protocolo en la cabecera en buffer de USB o ESP
 *
 */
void PutHeaderOnTx(_Tx *TX, uint8_t cmd, uint8_t CantDatos);
/*
 * Carga de checksum en buffer de USB o ESP
 *
 */
void PutcksOnTx(_Tx *TX);
/*
 * Carga de string en buffer de USB o ESP
 *
 */
void PutStrOnTx(_Tx *TX, const char *str);
/*
 * Captura de byte de buffer Rx de USB o ESP
 *
 */
uint8_t GetByteFromRx(_Rx *RX, int8_t pre, int8_t pos);
/*
 * Inicializacion de ESP8266
 * Se cargan comandos AT en el buffer Tx de ESP8266
 *
 */
void InitEsp(_Rx *RXUSART1);
/*
 * Decodifica comandos de ESP8266
 *
 */
void DecodeESP(_Rx *RXUSART1);

/*
 * Calculo de error cuadratico
 *
 */
void ErrorCuadratico();
/*
 * Cálculo de PID
 * Control de velocidad de motores
 *
 */
void PID();

void ConnectPC();

void SendUDPData();

void ADC();

void Reset();

void PutCIPSENDOnTx(const char * CantDatos);
/**********************************************************************************/
/**********************************************************************************/
/**********************************************************************************/

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**********************************************************************************/
/********************************** Funciones *************************************/
/**********************************************************************************/


void PID(){
//	uint16_t Valor;
//	uint8_t i;
//
//	for( i=0; i<8; i++){
//		Valor += bufADC[iAdc][i];
//	}
//
//	if(Valor < 1000){
//		Valor = 0;
//		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,0);
//		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,0);
//		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0);
//		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,0);
//		return;
//	}

/**************************** Cálculo de variables PID ****************************/
	ProporcionalMA.i16[0] = KpMA*error.i16[0];
	ProporcionalMB = KpMB*error.i16[0];
//	Integral += 2*error.i16; //Integral += Ki*error;
	DerivativoMA = KdMA*(error.i16[0]-lastError)*100; //Kd*(error-lastError)
	DerivativoMB = KdMB*(error.i16[0]-lastError)*100;

	PWM_A.i16[0]= ProporcionalMA.i16[0] + IntegralMA + DerivativoMA;
	PWM_B.i16[0]= ProporcionalMB + IntegralMB + DerivativoMB;

	PWMA.i16[0] = PWMBaseA.i16[0] + PWM_A.i16[0];
	PWMB.i16[0] = PWMBaseB.i16[0] - PWM_B.i16[0];
/**********************************************************************************/

/************************ Seteo de variables fuera de rango ************************/
	if(PWMA.i16[0] > PWMAMAX)
		PWMA.i16[0] = PWMAMAX;

//	if(PWMB.i16[0] > PWMBMAX+1000)
//		PWMB.i16[0] = PWMBMAX+1000;

	if(PWMA.i16[0] < 0)
		PWMA.i16[0] = 0;

	if(PWMB.i16[0] < 0)
		PWMB.i16[0] = 0;
/***********************************************************************************/

/*************************** Actuacion de motores con PID **************************/
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,PWMA.i16[0]);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,0);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,PWMB.i16[0]);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,0);
/***********************************************************************************/

	lastError=error.i16[0];
}

void ErrorCuadratico(){
/******************************* Variables locales *********************************/
	uint16_t aux[10];

	sensorValue = bufADC[iAdc][0];

	posMINCenter=0;
/***********************************************************************************/

/*************************** Encuentro la menor lectura ****************************/
	for(uint8_t c=0; c<8; c++){
		if(sensorValue < bufADC[iAdc][c]){
			sensorValue = bufADC[iAdc][c];
			posMINCenter = c;
		}
		aux[c+1]=bufADC[iAdc][c];
	}

	posMINCenter+=1;
	aux[0]=aux[2];
	aux[9]=aux[7];
/***********************************************************************************/

/***************** Calculo de error segun peso asignado a sensores *****************/
	posMINDerecha=posMINCenter+1;
	posMINIzquierda=posMINCenter-1;
	fx2_fx3=aux[posMINCenter]-aux[posMINDerecha];
	fx2_fx1=aux[posMINCenter]-aux[posMINIzquierda];
	x2_x1=COORD_SENSORES[posMINCenter]-COORD_SENSORES[posMINIzquierda];
	x2_x1cuad=(x2_x1*x2_x1);
	x2_x3=COORD_SENSORES[posMINCenter]-COORD_SENSORES[posMINDerecha];
	x2_x3cuad=(x2_x3*x2_x3);
	denominador=-(2*(x2_x1*fx2_fx3-x2_x3*fx2_fx1));
	if(denominador!= 0){
		error.i16[0]=(int16_t)COORD_SENSORES[posMINCenter]-( x2_x1cuad*fx2_fx3 - x2_x3cuad*fx2_fx1 ) / denominador;
	}
/***********************************************************************************/
}

void Reset(){
	espConnected = 0;
	ESPReadyToRecyb = 1;
	ResetESP = 1;
	HAL_UART_AbortReceive_IT(&huart1);
	HAL_GPIO_WritePin(GPIOB, RESET_ESP_Pin, GPIO_PIN_RESET);//Reset ESP8266
	DecodeTimeOut = 20;
}

void InitEsp(_Rx *RXUSART1){
	switch(Estado){
		case 0:
			PutStrOnTx((_Tx *)&TXUSART1,AT); //Envio comando AT para ver si responde el ESP
			PutStrOnTx((_Tx *)&TXUSB,AT);    // Depuracion por USB
		break;

		case 1:
			PutStrOnTx((_Tx *)&TXUSART1,CWMODE); //Envio comando CWMODE
			PutStrOnTx((_Tx *)&TXUSB,CWMODE);    // Depuracion por USB

		break;

		case 2:
			PutStrOnTx((_Tx *)&TXUSART1,CWJAP_); //Envio comando CWJAP con los datos de la red
			PutStrOnTx((_Tx *)&TXUSB,CWJAP_);    // Depuracion por USB

		break;

		case 3:
			PutStrOnTx((_Tx *)&TXUSART1,CIPMUX);
			PutStrOnTx((_Tx *)&TXUSB,CIPMUX);    // Depuracion por USB

		break;

		case 4:
			PutStrOnTx((_Tx *)&TXUSART1,CIPSTART);
			PutStrOnTx((_Tx *)&TXUSB,CIPSTART);    // Depuracion por USB

		break;
	}
	DecodeTimeOut = 4;
	ESPReadyToRecyb = 1;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
/***************** Contadores para control del flujo de tiempo ********************/
/*
 * Instancia cada 1ms
 *
 */
	if(htim->Instance == TIM4){
		Count100ms--;
		Countms--;
		Count5ms--;
		if(!Count100ms){
			Count100ms = 100;
			On100ms = 1;
		}

		if(RXUSB.header) {
			RXUSB.timeout--;
			if(!RXUSB.timeout)
				RXUSB.header = 0;
		}
	}
/**********************************************************************************/

/************************* Incio lectura analogica por DMA *************************/
/*
 * Instancia cada 500us
 *
 */
	if(htim->Instance == TIM3){
		HAL_ADC_Start_DMA(&hadc1, (uint32_t *) &bufADC[iAdc], 8);
	}
/***********************************************************************************/

}

void DecodeESP(_Rx *RXUSART1){

	if((espConnected) && (!DecodeHeaderESP)){
		//Decodifica \r\n+IPD,
		if((RXUSART1->Buff[RXUSART1->ir] == IPD[IndiceIPD]) && (IndiceIPD < 7)){
			IndiceIPD++;
			if(IndiceIPD == 7){
				DecodeIPD = 1;
				IndiceIPD = 0;
			}
		}
		else{
			if((IndiceIPD > 0) && (!DecodeIPD)){
				IndiceIPD = 0;
//				RXUSART1->ir = RXUSART1->iw;
				return;
			}
		}

		if(DecodeIPD){
			if(RXUSART1->Buff[RXUSART1->ir]==':'){
				DecodeHeaderESP = 1;
				DecodeIPD = 0;
				return;
			}
		}

		//Decodifica WIFI DISCONNECT\r\n
		if(RXUSART1->Buff[RXUSART1->ir] == WIFI_DISCONNECT[IndiceDisconnect]){
			IndiceDisconnect++;
			if(IndiceDisconnect == 17){
				IndiceDisconnect=0;
				Reset();
			}
		}
		else{
			if(IndiceDisconnect > 0){
				IndiceDisconnect = 0;
			}
		}

		//Decodifica ERROR
		if(RXUSART1->Buff[RXUSART1->ir] == Error[IndiceError]){
			IndiceError++;
			if(IndiceError == 9){
				IndiceError=0;
				Reset();
			}
		}
		else{
			if(IndiceError > 0){
				IndiceError = 0;
			}
		}

		RXUSART1->ir++;
	}

	//Decodifica inicializacion de ESP8266
	if((!espConnected) && (!DecodeTimeOut)){
		switch(Estado){
			case 0:
				if(RXUSART1->Buff[RXUSART1->ir]==ANS_AT[Indice]){
					Indice ++;
					RXUSART1->ir++;
					if(Indice == 10){
						Indice = 0;
						Estado = 1;
						ESPReadyToRecyb = 0;
					}
				}
				else{
					RXUSART1->ir = RXUSART1->iw;
					Indice = 0;
					ESPReadyToRecyb = 0;
				}
			break;

			case 1:
				if(RXUSART1->Buff[RXUSART1->ir]==ANS_CWMODE[Indice]){
					Indice++;
					RXUSART1->ir++;
					if(Indice==19){
						Indice=0;
						Estado = 2;
						ESPReadyToRecyb = 0;
					}
				}
				else{
					if(Indice>0){
						RXUSART1->ir = RXUSART1->iw;
						Indice=0;
					}
					Estado = 0;
				}
			break;

			case 2:
				if(RXUSART1->Buff[RXUSART1->ir]==ANS_CWJAP_[Indice]){
					Indice++;
					RXUSART1->ir++;
					if(Indice==LONG_ANS_CWJAP_){
						Indice = 0;
						Estado = 3;
						ESPReadyToRecyb = 0;
					}
				}
				else{
					if(Indice>0){
						RXUSART1->ir = RXUSART1->iw;
						Indice=0;
					}
					Estado = 0;
				}
			break;

			case 3:
					if(RXUSART1->Buff[RXUSART1->ir]==ANS_CIPMUX[Indice]){
						Indice++;
						RXUSART1->ir++;
						if(Indice == 19){
							Indice = 0;
							Estado = 4;
							ESPReadyToRecyb = 0;
						}
					}
					else{
						if(Indice>0){
							RXUSART1->ir = RXUSART1->iw;
							Indice = 0;
						}
						Estado = 0;
						ESPReadyToRecyb = 0;
					}
			break;

			case 4:
					if(RXUSART1->Buff[RXUSART1->ir]==ANS_CIPSTART[Indice]){
						Indice++;
						RXUSART1->ir++;
						if(Indice == LONG_ANS_CIPSTART ){
							Indice = 0;
							espConnected = 1;
							Estado = 0;
							Countms = 70;
						}
					}
					else{
						if(Indice>0){
							RXUSART1->ir = RXUSART1->iw;
							Indice = 0;
							espConnected = 1;
							Countms = 70;
							Estado = 0;
						}
						Estado = 0;
					}
			break;
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART1){
//		TXUSB.Buff[TXUSB.iw++] = rxUSART1Buff[RXUSART1.iw];
		RXUSART1.iw++;
		HAL_UART_Receive_IT(&huart1, &rxUSART1Buff[RXUSART1.iw], 1);
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){

	// Incrento indice del buffer
	iAdc++;
	if(iAdc == 32){
		iAdc = 0;
	}
//	ADC();
}
void ADC(){
	volatile uint8_t c;

/***********************************************************************************/
/********************* Media movil de las 3 primeras lecturas **********************/
	if(FirtScan){
		switch(iAdc){
			case 1:
				for(c=0;c<8;c++){
					bufADC[1][c]=(uint16_t)((bufADC[0][c]+bufADC[1][c]) / 2 );
				}
			break;

			case 2:
				for(c=0;c<8;c++){
					bufADC[2][c]=(uint16_t)((bufADC[0][c]+bufADC[1][c]+bufADC[2][c]) / 3);
				}
			break;

			case 3:
				for(c=0;c<8;c++){
					bufADC[3][c]=(uint16_t)((bufADC[0][c]+bufADC[1][c]+bufADC[2][c]+bufADC[3][c]) / 4);
				}
				FirtScan = 0;
			break;
		}
	}

/***********************************************************************************/
/********************* Media movil de las siguientes lecturas **********************/
	else{
		switch(iAdc){
			// Cruce por 0 y 1 donde se toman dos valores anteriores para hacer la media
			case 0:
				for(c = 0; c<8; c++){
					bufADC[0][c]=(uint16_t)(( bufADC[0][c]+bufADC[31][c]+bufADC[30][c]+bufADC[29][c]) / 4) ;
				}
			break;

			case 1:
				for(c = 0; c<8; c++){
					bufADC[1][c]=(uint16_t)((bufADC[0][c]+bufADC[1][c]+bufADC[31][c]+bufADC[30][c]) / 4);
				}
			break;

			case 2:
				for(c = 0; c<8; c++){
					bufADC[2][c]=(uint16_t)((bufADC[2][c]+bufADC[1][c]+bufADC[0][c]+bufADC[31][c]) / 4);
				}
			break;

			default:
				// Cálculo comprendido entre bufADC[2] y bufADC[31]
				for(c = 0; c<8; c++){
					bufADC[iAdc][c] = (uint16_t)((bufADC[iAdc-3][c] + bufADC[iAdc-2][c] + bufADC[iAdc-1][c] + bufADC[iAdc][c])/4);
				}
			break;
		}
	}
/***********************************************************************************/
}
void MyCallBackOnUSBData(uint8_t *buf, uint32_t len){
	for(uint32_t i=0; i<len; i++){
		rxUSBBuff[RXUSB.iw++] = buf[i];
	}
}

void DecodeHeader(_Rx *RX)
{
    uint8_t i;
    i = RX->iw;

    while(RX->ir != i) {
        switch(RX->header) {
            case 0:
                if(RX->Buff[RX->ir] == 'U') {
                    RX->header = 1;
                    RX->timeout = 5;
                }
                break;
            case 1:
                if(RX->Buff[RX->ir] == 'N') {
                    RX->header = 2;
                } else {
                    RX->header = 0;
                    RX->ir --;
                }
                break;
            case 2:
                if(RX->Buff[RX->ir] == 'E') {
                    RX->header = 3;
                } else {
                    RX->header = 0;
                    RX->ir --;
                }
                break;
            case 3:
                if(RX->Buff[RX->ir] == 'R') {
                    RX->header = 4;
                } else {
                    RX->header = 0 ;
                    RX->ir --;
                }
                break;
            case 4:
                RX->nbytes = RX->Buff[RX->ir];
                RX->header = 5;
                break;
            case 5:
                if(RX->Buff[RX->ir] == ':') {
                    RX->header = 6;
                    RX->iData = RX->ir + 1;
                    RX->iData &= RX->maskSize;
                    RX->cks = 'U' ^ 'N' ^ 'E' ^ 'R' ^ ':' ^ RX->nbytes;

                } else {
                    RX->header = 0 ;
                    RX->ir --;
                }
                break;
            case 6:
                RX->nbytes--;
                if(RX->nbytes > 0) {
                    RX->cks ^= RX->Buff[RX->ir];
                } else {
                    RX->header = 0 ;
                    if(RX->cks == RX->Buff[RX->ir]) {
                        RX->ISCMD = 1;
                        DecodeHeaderESP = 0;
                    }
                }
                break;

            default:
                RX->header = 0;
                DecodeHeaderESP = 0;
                break;
        }
        RX->ir &= RX->maskSize;
        RX->ir ++;
        RX->ir &= RX->maskSize;    // Enmascaro el indice de lectura - Cuando llega a 255 se pone en 0
    }
}

void DecodeCmd(_Rx *RX, _Tx *TX){
    RX->ISCMD = 0;

    switch(RX->Buff[RX->iData]){

    	case 0xA7:
		    PCConnected = 1;
			Data = 0;
    	break;

    	case 0xA6:
			PCConnected = 0;
			Data = 0;
			Start = 0;
		break;

    	/*
         * Comando para enviar alive
         */
        case 0xF0:                                                              //Alive
			if(!espConnected){
	        	PutHeaderOnTx((_Tx *)&TXUSB, 0xF0, 2);
	            PutByteOnTx((_Tx *)&TXUSB, 0x0D);
	            PutcksOnTx((_Tx *)&TXUSB);
			}
		break;
		/*
		 * Comando para enviar valor de sensores
		 */
        case 0xA0:
        	if(!espConnected){
        		PutHeaderOnTx((_Tx *)&TXUSB, 0xA0, 17);
        		PutBuffOnTx((_Tx *)&TXUSB, (uint8_t *)bufADC, 16);
				PutcksOnTx((_Tx *)&TXUSB);
        	}
        break;
        /*
		 * Comando para recibir valor de constantes
		 */
        case 0xE0:
        	KpMA =  GetByteFromRx((_Rx *)&RXUSART1, 1, 0);
        	KdMA =  GetByteFromRx((_Rx *)&RXUSART1, 1, 0);

        	KpMB =  GetByteFromRx((_Rx *)&RXUSART1, 1, 0);
			KdMB =  GetByteFromRx((_Rx *)&RXUSART1, 1, 0);
        break;
        /*
		 * Comando para recibir valor base o minimo de PWM
		 */
        case 0xA1:

        	PWMBaseA.i8[0] = GetByteFromRx((_Rx *)&RXUSART1, 1, 0);
        	PWMBaseA.i8[1] = GetByteFromRx((_Rx *)&RXUSART1, 1, 0);

        	PWMBaseB.i8[0] = GetByteFromRx((_Rx *)&RXUSART1, 1, 0);
        	PWMBaseB.i8[1] = GetByteFromRx((_Rx *)&RXUSART1, 1, 0);

			RespMotor = 1;

        	if(!espConnected){
                PutHeaderOnTx(&TXUSB, 0xA1, 2);
    			PutByteOnTx(&TXUSB, 0x0D);
    			PutcksOnTx(&TXUSB);
            }

        break;

        case 0xA2:
		  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
		  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
		  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
		  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
		  Start = 0;
        break;

        case 0xA3:
		  Start = 1;
		break;


        /*
         * Comando para enviar aviso de error
         */
        default:
			if(!espConnected){
	        	PutHeaderOnTx((_Tx *)&TXUSB, 0xFF, 1);
	            PutcksOnTx((_Tx *)&TXUSB);
			}
        break;
    }
}

void PutCIPSENDOnTx(const char * CantDatos){
	PutStrOnTx((_Tx *)&TXUSART1, CIPSEND);
	PutStrOnTx((_Tx *)&TXUSART1, CantDatos);
	TXUSART1.Buff[TXUSART1.iw++] = '\r';
	TXUSART1.iw &= TXUSART1.maskSize;
	TXUSART1.Buff[TXUSART1.iw++] = '\n';
	TXUSART1.iw &= TXUSART1.maskSize;
}

void SendUDPData(){
	switch(Data){
		case 0:
			PutCIPSENDOnTx("42");
			Countms = 20;
			Data++;
		break;

		case 1:
			PutHeaderOnTx((_Tx *)&TXUSART1, 0xA8, 35);

			PutBuffOnTx((_Tx *)&TXUSART1, (uint8_t *)bufADC, 16);

			TXUSART1.Buff[TXUSART1.iw++] = error.i8[0];
			TXUSART1.iw &= TXUSART1.maskSize;
			TXUSART1.Buff[TXUSART1.iw++] = error.i8[1];
			TXUSART1.iw &= TXUSART1.maskSize;

			TXUSART1.Buff[TXUSART1.iw++] = PWMA.i8[0];
			TXUSART1.iw &= TXUSART1.maskSize;
			TXUSART1.Buff[TXUSART1.iw++] = PWMA.i8[1];
			TXUSART1.iw &= TXUSART1.maskSize;

			TXUSART1.Buff[TXUSART1.iw++] = PWMB.i8[0];
			TXUSART1.iw &= TXUSART1.maskSize;
			TXUSART1.Buff[TXUSART1.iw++] = PWMB.i8[1];
			TXUSART1.iw &= TXUSART1.maskSize;

			TXUSART1.Buff[TXUSART1.iw++] = PWM_A.i8[0];
			TXUSART1.iw &= TXUSART1.maskSize;
			TXUSART1.Buff[TXUSART1.iw++] = PWM_A.i8[1];
			TXUSART1.iw &= TXUSART1.maskSize;

			TXUSART1.Buff[TXUSART1.iw++] = PWM_B.i8[0];
			TXUSART1.iw &= TXUSART1.maskSize;
			TXUSART1.Buff[TXUSART1.iw++] = PWM_B.i8[1];
			TXUSART1.iw &= TXUSART1.maskSize;

			TXUSART1.Buff[TXUSART1.iw++] = PWMBaseA.i8[0];
			TXUSART1.iw &= TXUSART1.maskSize;
			TXUSART1.Buff[TXUSART1.iw++] = PWMBaseA.i8[1];
			TXUSART1.iw &= TXUSART1.maskSize;

			TXUSART1.Buff[TXUSART1.iw++] = PWMBaseB.i8[0];
			TXUSART1.iw &= TXUSART1.maskSize;
			TXUSART1.Buff[TXUSART1.iw++] = PWMBaseB.i8[1];
			TXUSART1.iw &= TXUSART1.maskSize;

			TXUSART1.Buff[TXUSART1.iw++] = ProporcionalMA.i8[0];
			TXUSART1.iw &= TXUSART1.maskSize;
			TXUSART1.Buff[TXUSART1.iw++] = ProporcionalMA.i8[1];
			TXUSART1.iw &= TXUSART1.maskSize;

			PutByteOnTx((_Tx *)&TXUSART1, 0xF0);
			PutByteOnTx((_Tx *)&TXUSART1, 0x0D);

			PutcksOnTx((_Tx *)&TXUSART1);

			if(RespMotor){
				Data++;
				Countms = 30;
			}
			else{
				Countms = 180;
				Data=0;
			}
		break;

		case 2:
			PutCIPSENDOnTx("9");
			Countms = 30;
			Data++;
		break;

		case 3:
			PutHeaderOnTx((_Tx *)&TXUSART1, 0xA1, 2);
			PutByteOnTx((_Tx *)&TXUSART1, 0x0D);
			PutcksOnTx((_Tx *)&TXUSART1);
			Countms = 120;
			RespMotor = 0;
			Data = 0;
		break;

	}
}


void ConnectPC(){
	switch(Data){
		case 0:
			PutCIPSENDOnTx("8");
			Countms = 20;
			Data++;
		break;

		case 1:
			PutHeaderOnTx((_Tx *)&TXUSART1, 0xA7, 1);
			PutcksOnTx((_Tx *)&TXUSART1);
			Countms = 780;
			Data = 0;
		break;
	}
}

uint8_t GetByteFromRx(_Rx *RX, int8_t pre, int8_t pos){
    uint8_t aux;

    RX->iData += pre;
    RX->iData &= RX->maskSize;
    aux = RX->Buff[RX->iData];
    RX->iData += pos;
    RX->iData &= RX->maskSize;

    return aux;
}

void PutStrOnTx(_Tx *TX, const char *str)
{
    uint8_t i = 0;

    while(str[i]) {
        TX->Buff[TX->iw++] = str[i++];
        TX->iw &= TX->maskSize;
    }
}

void PutByteOnTx(_Tx *TX, uint8_t value)
{
    TX->Buff[TX->iw++] = value;
    TX->iw &= TX->maskSize;
}

void PutBuffOnTx(_Tx *TX, uint8_t *buf, uint8_t length)
{
    uint8_t i;

    for(i=0; i<length; i++) {

    	TX->Buff[TX->iw++] = buf[i];
    	TX->iw &= TX->maskSize;
    }
}

void PutHeaderOnTx(_Tx *TX, uint8_t cmd, uint8_t CantDatos)
{
    TX->Buff[TX->iw++] = 'U';
    TX->iw &= TX->maskSize;
    TX->Buff[TX->iw++] = 'N';
    TX->iw &= TX->maskSize;
    TX->Buff[TX->iw++] = 'E';
    TX->iw &= TX->maskSize;
    TX->Buff[TX->iw++] = 'R';
    TX->iw &= TX->maskSize;
    //lcmd cantidad de datos: id+payload+cks
    TX->length = CantDatos;
    TX->Buff[TX->iw++] = CantDatos + 1;
    TX->iw &= TX->maskSize;
    TX->Buff[TX->iw++] = ':';
    TX->iw &= TX->maskSize;
    TX->Buff[TX->iw++] = cmd;
    TX->iw &= TX->maskSize;
}

void PutcksOnTx(_Tx *TX)
{
    uint8_t cks, i;

    cks = 0;
    i = TX->length + 6;
    i = TX->iw - i;
    i &= TX->maskSize;
    while(i != TX->iw) {
        cks ^= TX->Buff[i++];
        i &= TX->maskSize;
    }

    TX->Buff[TX->iw++] = cks;
    TX->iw &= TX->maskSize;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  CDC_AttachOnUSBData(MyCallBackOnUSBData);

  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Base_Start_IT(&htim3);
  //Enable ESP8266
  HAL_GPIO_WritePin(GPIOB, ENABLE_ESP_Pin, GPIO_PIN_SET);	//Enable ESP8266
  HAL_GPIO_WritePin(GPIOB, RESET_ESP_Pin, GPIO_PIN_SET);	//Reset ESP8266
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
/***********************************************************************************/
/************************** Inicializacion de contadores ***************************/
/***********************************************************************************/
  Count100ms = 100;
  Countms = 0;
  Count5ms = 0;
  Count500ms = 5;
  DecodeTimeOut = 60;
/***********************************************************************************/
/***********************************************************************************/
/***********************************************************************************/

/***********************************************************************************/
/*************************** Inicializacion de variables USB ***********************/
/***********************************************************************************/
  RXUSB.Buff = (uint8_t *)rxUSBBuff;
  RXUSB.iw = 0;
  RXUSB.ir = 0;
  RXUSB.header = 0;
  RXUSB.maskSize = 255;
  RXUSB.ISCMD = 0;

  TXUSB.Buff = txUSBBuff;
  TXUSB.iw = 0;
  TXUSB.ir = 0;
  TXUSB.maskSize = 255;
  TXUSB.maskBuf = 255;
/***********************************************************************************/
/***********************************************************************************/
/***********************************************************************************/

/***********************************************************************************/
/********************** Inicialización de  ESP8266 ************************/
/***********************************************************************************/
  RXUSART1.Buff = (uint8_t *)rxUSART1Buff;
  RXUSART1.iw = 0;
  RXUSART1.ir = 0;
  RXUSART1.header = 0;
  RXUSART1.maskSize = 255;
  RXUSART1.ISCMD = 0;

  TXUSART1.Buff = txUSART1Buff;
  TXUSART1.iw = 0;
  TXUSART1.ir = 0;
  TXUSART1.maskSize = 255;
  TXUSART1.maskBuf = 255;

  IndiceError = 0;
  IndiceIPD = 0;

  /*Bits de control*/
  DecodeIPD = 0;
  DecodeCIPSEND = 0;
  DecodeHeaderESP = 0;
  ESPReadyToRecyb = 0;
  SendADC = 0;
  ResetESP = 1;
  Estado = 0;
  Indice = 0;
  SentDataESP = 0;
  RespMotor = 0;
  SendAlive = 0;
  PCConnected = 0;
  Start = 0;

/***********************************************************************************/
/***********************************************************************************/
/***********************************************************************************/

/***********************************************************************************/
/************************ Inicializacion de variables ADC **************************/
/***********************************************************************************/
  iAdc = 0;
  FirtScan=1;
/***********************************************************************************/
/***********************************************************************************/
/***********************************************************************************/


/***********************************************************************************/
/***************** Inicializacion de variables error cuadratico ********************/
/***********************************************************************************/
  posMINCenter = 0;
  posMINDerecha = 0;
  posMINIzquierda = 0;
  sensorValue = 0;
  error.i8[0] = 0;
/***********************************************************************************/
/***********************************************************************************/
/***********************************************************************************/

/***********************************************************************************/
/************************* Inicialización de variables PID *************************/
/***********************************************************************************/
  KpMA = KiMA = KdMA = KpMB = KiMB = KdMB = 0;
  IntegralMA = DerivativoMA = ProporcionalMB = IntegralMB = DerivativoMB = 0;
  ProporcionalMA.i16[0] = 0;
  PWMA.i16[0] = 0;
  PWMB.i16[0] = 0;
  PWM_A.i16[0] = 0;
  PWM_B.i16[0] = 0;
  PWMBaseA.i16[0] = 0;
  PWMBaseB.i16[0] = 0;
/***********************************************************************************/
/***********************************************************************************/
/***********************************************************************************/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //LED de estado - Parpadea cada 100ms
	  if(On100ms){
		  On100ms = 0;
		  HAL_GPIO_TogglePin(LED13_GPIO_Port, LED13_Pin);

		  //Utilizado como delay para no capturar basura de la ESP8266 al iniciarla
		  if(DecodeTimeOut > 0){
			  DecodeTimeOut--;
		  }
	  }

	  if((espConnected) && (!DecodeHeaderESP) && (!RXUSART1.ISCMD) && (!Countms)){
		  if(PCConnected)SendUDPData();
		  else ConnectPC();

	  }

	  if((Start) && (!Count5ms)){
		  ErrorCuadratico();
		  PID();
		  Count5ms=10;
	  }

	  //Recepcion por USB - Decodifica header
	  if(RXUSB.iw != RXUSB.ir) {
		  DecodeHeader((_Rx *)&RXUSB);
	  }
	  //Recepcion por USB - Decodifica comando
	  if(RXUSB.ISCMD) {
		  DecodeCmd((_Rx *)&RXUSB, (_Tx *)&TXUSB);
	  }
	  //Transmision por USB
	  if(TXUSB.iw != TXUSB.ir) {
		  if(TXUSB.iw > TXUSB.ir){
			  if(USBD_OK==CDC_Transmit_FS(&TXUSB.Buff[TXUSB.ir], (TXUSB.iw-TXUSB.ir)))
				  TXUSB.ir=TXUSB.iw;
		  }
		  else{
			  w.u16[0] = (TXUSB.maskBuf+1)-TXUSB.ir;
			  if(USBD_OK==CDC_Transmit_FS(&TXUSB.Buff[TXUSB.ir], w.u16[0]))
				  TXUSB.ir = 0;
		  }
		  TXUSB.ir &= TXUSB.maskSize;
	  }

	  //Primera inicializacion de ESP8266 y Reset
	  if((!DecodeTimeOut) && (ResetESP) && (!ESPReadyToRecyb)){
		  InitEsp((_Rx *)&RXUSART1);
		  //Interrupcion para recibir datos desde ESP8266
		  HAL_UART_Receive_IT(&huart1,rxUSART1Buff,1);
		  ResetESP = 0;
	  }

	  //Continua con los casos para iniciar
	  if((!espConnected) && (!ESPReadyToRecyb) && (!ResetESP) && (!DecodeTimeOut)){
		  InitEsp((_Rx *)&RXUSART1);
	  }

	  //Recepcion por ESP8266 - Decodifica comandos provenientes de ESP8266
	  if(RXUSART1.iw != RXUSART1.ir){
		  DecodeESP((_Rx *)&RXUSART1);
	  }
	  //Si la decodificacion resulta de un dato proveniente de PC, se decodifica header
	  if(DecodeHeaderESP){
		  DecodeHeader((_Rx *)&RXUSART1);

	  }
	  //Recepcion por ESP8266 - Decodifica comando
	  if(RXUSART1.ISCMD) {
		  DecodeCmd((_Rx *)&RXUSART1, (_Tx *)&TXUSART1);

	  }

	  //Transmision por ESP8266
	  if(TXUSART1.iw != TXUSART1.ir) {
		  if((huart1.Instance->SR & UART_FLAG_TXE) == UART_FLAG_TXE){
			  huart1.Instance->DR = txUSART1Buff[TXUSART1.ir++];
		  }
	  }

	  //Reset ESPE8266
	  if((HAL_GPIO_ReadPin(GPIOB, RESET_ESP_Pin) == GPIO_PIN_RESET) && (!DecodeTimeOut) && (ResetESP) && (ESPReadyToRecyb)){
		  HAL_GPIO_WritePin(GPIOB, RESET_ESP_Pin, GPIO_PIN_SET);
		  DecodeTimeOut = 1;
		  ESPReadyToRecyb = 0;
		  Estado = 2;
	  }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 8;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 18000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 7;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 9000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED13_GPIO_Port, LED13_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RESET_ESP_Pin|ENABLE_ESP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED13_Pin */
  GPIO_InitStruct.Pin = LED13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED13_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA4 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RESET_ESP_Pin ENABLE_ESP_Pin */
  GPIO_InitStruct.Pin = RESET_ESP_Pin|ENABLE_ESP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
