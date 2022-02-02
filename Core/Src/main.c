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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define ADCTIMEOUT      flag1.bit.b0
/************************* Valores MAX-MIN de motores *****************************/
#define PWMBMAX			4800
#define PWMAMAX			9000
#define PWMBMIN			2800
#define PWMAMIN			4500
/**********************************************************************************/

/************************* Longitud de respuestas ESP *****************************/
#define LONG_CWJAP_MICROS	35
#define LONG_ANS_CWJAP_MICROS 70
#define LONG_ANS_CIFSR 141
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
const char CWJAP_MICROS[]="AT+CWJAP=\"MICROS\",\"micros1234567\"\r\n";
const char CIFSR[] = "AT+CIFSR\r\n";
const char CIPMUX[] = "AT+CIPMUX=0\r\n";
const char CIPSTART[]="AT+CIPSTART=\"UDP\",\"192.168.0.5\",30011,3011\r\n";
const char CIPSEND[] = "AT+CIPSEND=";
const char CIPCLOSE[] = "AT+CIPCLOSE\r\n";
const char CWQAP[] = "AT+CWQAP\r\n";
const char CWMODE[] = "AT+CWMODE=3\r\n";
/********************************************************************************/

/*************************** Respuestas comandos AT *****************************/
const char ANS_CWMODE[] = "AT+CWMODE=3\r\n\r\nOK\r\n"; //19
const char ANS_CWQAP[] = "AT+CWQAP\r\n\r\nOK\r\n"; //16
const char ANS_CWJAP_MICROS[]="AT+CWJAP=\"MICROS\",\"micros1234567\"\r\nWIFI CONNECTED\r\nWIFI GOT IP\r\n\r\nOK\r\n";//70
const char ANS_CIPMUX[] = "AT+CIPMUX=0\r\n\r\nOK\r\n"; //19
const char ANS_CIFSR[] = "AT+CIFSR\r\n+CIFSR:APIP,\"192.168.4.1\"\r\n+CIFSR:APMAC,\"4a:3f:da:55:28:71\"\r\n+CIFSR:STAIP,\"192.168.1.15\"\r\n+CIFSR:STAMAC,\"48:3f:da:55:28:71\"\r\n\r\nOK\r\n";//141
const char ANS_CIPSTART[]="AT+CIPSTART=\"UDP\",\"192.168.1.5\",30011,3011\r\nCONNECT\r\n\r\nOK\r\n";//59
const char ANS_CIPSEND[] = {};
const char ANS_AT[] = "AT\r\n\r\nOK\r\n";
const char AUTOMATIC_WIFI_CONNECTED[] = {"WIFI CONNECTED\r\nWIFI GOT IP\r\n"};
const char WIFI_DISCONNECTED[] = "WIFI DISCONNECT\r\n";
const char CIFSR_STAIP[] = "+CIFSR:STAOP,";
const char OK[]="\r\nOK\r\n";
const char CIPSEND1[]={'A','T','+','C','I','P','S','E','N','D','='};
const char CIPSEND2[]={'\r','\n','\r','\n','O','K','\r','\n','>',};
const char CIPSEND3[]="Recv";
const char CIPSEND4[]={ "bytes\r\n\r\nSEND OK\r\n"};
/*********************************************************************************/

/********************* Cálculo error algoritmo cuadratico ************************/
const int COORD_SENSORES[]={-4,-3,-2,-1,1,2,3,4};
static uint8_t FirtScan;
volatile uint16_t bufADC[32][8];
volatile uint8_t iAdc;
uint8_t posMINCenter, posMINRight, posMINLeft;
uint16_t sensorValue;
float error,fx2_fx3,fx2_fx1,x2_x1,x2_x3,x2_x1cuad,x2_x3cuad,denominador;
/*********************************************************************************/
/************************************* PID ***************************************/
volatile float Kp, Ki, Kd, Proporcional, Integral, Derivativo, PWM, lastError;
volatile uint8_t TimeOutPID;
/*********************************************************************************/

/***************************** Contadores timmer *********************************/
volatile uint16_t timeOutms, On100ms, On200ms, On3000ms, On500ms, Count100ms, Count200ms, Count3000ms, Count500ms;
/*********************************************************************************/

/********************** Comunicación - Buffer e indices  *************************/
volatile _Rx RXUSB, RXUSART1;
_Tx TXUSB, TXUSART1;
uint8_t rxUSBBuff[256], rxUSART1Buff[256];
uint8_t txUSBBuff[256], txUSART1Buff[256];
/*********************************************************************************/
_work w;

volatile _sFlag flag1;


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
void DecodeCmd(_Rx *RX);
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
void PutHeaderOnTx(_Tx *TX, uint8_t id, uint8_t lcmd);
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
 * Inicializacin de ESP
 * Se envia comando AT
 * Pasado tiempo correspondiente se decodifica respuesta
 *
 */
void ESP(_Rx *RXUSART1);
/*
 * Decodifica respuesta entrante hasta encontrar caracter "K" de "OK"
 *
 */
uint8_t DecodeANS(_Rx *RX, const char *str, uint8_t strlen);
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
void PID(uint32_t PWMA,uint32_t PWMB);

/**********************************************************************************/
/**********************************************************************************/
/**********************************************************************************/

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**********************************************************************************/
/********************************** Funciones *************************************/
/**********************************************************************************/

void PID(uint32_t PWMA,uint32_t PWMB){

/**************************** Cálculo de variables PID ****************************/
	Proporcional = Kp*error;
	Integral += Ki*TimeOutPID*error;
	Derivativo = Kd*(error-lastError)/TimeOutPID;

	PWM= Proporcional + Integral + Derivativo;

	PWMA = PWMA - PWM;
	PWMB = PWMB + PWM;
/**********************************************************************************/

/************************ Seteo de variables fuera de rango ************************/
	if(PWMA>PWMAMAX)
		PWMA=PWMAMAX;
	if(PWMB>PWMBMAX)
		PWMB=PWMBMAX;
	if(PWMA<PWMAMIN)
		PWMA=PWMAMIN;
	if(PWMB>PWMBMIN)
		PWMB=PWMBMIN;
/***********************************************************************************/

/*************************** Actuacion de motores con PID **************************/
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,PWMA);
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,0);
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,0);
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,PWMB);
/***********************************************************************************/

	lastError=error;
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
	posMINRight=posMINCenter-1;
	posMINLeft=posMINCenter+1;
	fx2_fx3=aux[posMINCenter]-aux[posMINRight];
	fx2_fx1=aux[posMINCenter]-aux[posMINLeft];
	x2_x1=COORD_SENSORES[posMINCenter]-COORD_SENSORES[posMINLeft];
	x2_x1cuad=(x2_x1*x2_x1);
	x2_x3=COORD_SENSORES[posMINCenter]-COORD_SENSORES[posMINRight];
	x2_x3cuad=(x2_x3*x2_x3);
	denominador=(2*(x2_x1*fx2_fx3-x2_x3*fx2_fx1));
	if(denominador!= 0){
		error=COORD_SENSORES[posMINCenter]-( x2_x1cuad*fx2_fx3 - x2_x3cuad*fx2_fx1 ) / denominador;
	}
/***********************************************************************************/
}

void ESP(_Rx *RXUSART1){
	uint8_t aux,i=0;
	aux = RXUSART1->iw;
	while(RXUSART1->ir != RXUSART1->iw) {
		switch(RXUSART1->state){
			case 0: //Envio comando
				PutStrOnTx(&TXUSART1,AT); //Envio comando AT para ver si responde el ESP
				PutStrOnTx(&TXUSB,AT);    // Depuracion por USB
				RXUSART1->state = 1;
				Count500ms = 5;
			break;

			case 1: //Leo respuesta
				if(On500ms){
					if(i==0){
						PutStrOnTx(&TXUSB,ANS_AT);    // Depuracion por USB	solo de prueba
					}
					if(RXUSART1->Buff[RXUSART1->ir]==ANS_AT[i]){
						i++;
						if(i==10){
							i=0;
							RXUSART1->state = 2;
							On500ms = 0;
						}
					}
					else{
						if(i>0){
							RXUSART1->ir = aux;
							i=0;
						}
						RXUSART1->state = 0;
					}
					RXUSART1->ir++;
					if(i==0){
						PutStrOnTx(&TXUSB,ANS_AT);    // Depuracion por USB	solo de prueba
					}
				}
			break;

			case 2: //Envio comando
				PutStrOnTx(&TXUSART1,CWMODE); //Envio comando CWMODE
				PutStrOnTx(&TXUSB,CWMODE);    // Depuracion por USB
				RXUSART1->state = 3;
				Count500ms = 5;
			break;

			case 3: //Leo respuesta
				if(On500ms){
					if(i==0){
						PutStrOnTx(&TXUSB,ANS_CWMODE);    // Depuracion por USB
					}
					if(RXUSART1->Buff[RXUSART1->ir]==ANS_CWMODE[i]){
						i++;
						if(i==19){
							i=0;
							RXUSART1->state = 4;
							On500ms = 0;
						}
					}
					else{
						if(i>0){
							RXUSART1->ir = aux;
							i=0;
						}
						RXUSART1->state = 2;
					}
					RXUSART1->ir++;
					if(i==0){
						PutStrOnTx(&TXUSB,ANS_CWMODE);    // Depuracion por USB
					}
				}
			break;

			case 4: //Envio comando
				PutStrOnTx(&TXUSART1,CWJAP_MICROS); //Envio comando CWJAP con los datos de la red
				PutStrOnTx(&TXUSB,CWJAP_MICROS);    // Depuracion por USB
				RXUSART1->state = 5;
				Count3000ms = 30;
			break;

			case 5: //Leo respuesta
				if(On3000ms){
					if(i==0){
						PutStrOnTx(&TXUSB,ANS_CWJAP_MICROS);    // Depuracion por USB
					}
					if(RXUSART1->Buff[RXUSART1->ir]==ANS_CWJAP_MICROS[i]){
						i++;
						if(i==LONG_ANS_CWJAP_MICROS){
							i=0;
							RXUSART1->state = 6;
							On3000ms = 0;
						}
					}
					else{
						if(i>0){
							RXUSART1->ir = aux;
							i=0;
						}
						RXUSART1->state = 4;
					}
					RXUSART1->ir++;
					if(i==0){
						PutStrOnTx(&TXUSB,ANS_CWJAP_MICROS);    // Depuracion por USB
					}
				}
			break;

			case 6: //Envio comando
				PutStrOnTx(&TXUSART1,CIFSR); //Envio comando AT para ver si responde el ESP
				PutStrOnTx(&TXUSB,CIFSR);    // Depuracion por USB
				RXUSART1->state = 7;
				Count500ms = 5;
			break;

			case 7:
				if(On500ms){
					if(i==0){
						PutStrOnTx(&TXUSB,ANS_CIFSR);    // Depuracion por USB
					}
					if(RXUSART1->Buff[RXUSART1->ir]==ANS_CIFSR[i]){
						i++;
						if(i==LONG_ANS_CIFSR){
							i=0;
							RXUSART1->state = 8;
							On500ms = 0;
						}
					}
					else{
						if(i>0){
							RXUSART1->ir = aux;
							i=0;
						}
						RXUSART1->state = 6;
					}
					RXUSART1->ir++;
					if(i==0){
						PutStrOnTx(&TXUSB,ANS_CIFSR);    // Depuracion por USB
					}
				}
			break;
			case 8:	//Envio comando
				PutStrOnTx(&TXUSART1,CIPMUX); //Envio comando AT para ver si responde el ESP
				PutStrOnTx(&TXUSB,CIPMUX);    // Depuracion por USB
				RXUSART1->state = 9;
				Count500ms = 5;
			break;

			case 9: //Leo respuesta
				if(On500ms){
					if(i==0){
						PutStrOnTx(&TXUSB,ANS_CIPMUX);    // Depuracion por USB
					}
					if(RXUSART1->Buff[RXUSART1->ir]==ANS_CIPMUX[i]){
						i++;
						if(i==19){
							i=0;
							RXUSART1->state = 10;
							On500ms = 0;
						}
					}
					else{
						if(i>0){
							RXUSART1->ir = aux;
							i=0;
						}
						RXUSART1->state = 8;
					}
					RXUSART1->ir++;
					if(i==0){
						PutStrOnTx(&TXUSB,ANS_CIPMUX);    // Depuracion por USB
					}
				}
			break;

			case 10:	//Envio comando
				PutStrOnTx(&TXUSART1,CIPSTART); //Envio comando AT para ver si responde el ESP
				PutStrOnTx(&TXUSB,CIPSTART);    // Depuracion por USB
				RXUSART1->state = 11;
				Count3000ms = 30;
			break;

			case 11: //Leo respuesta
				if(On3000ms){
					if(i==0){
						PutStrOnTx(&TXUSB,ANS_CIPSTART);    // Depuracion por USB
					}
					if(RXUSART1->Buff[RXUSART1->ir]==ANS_CIPSTART[i]){
						i++;
						if(i==59){
							i=0;
							RXUSART1->state = 12;
							On3000ms = 0;
						}
					}
					else{
						if(i>0){
							RXUSART1->ir = aux;
							i=0;
						}
						RXUSART1->state = 10;
					}
					RXUSART1->ir++;
					if(i==0){
						PutStrOnTx(&TXUSB,ANS_CIPSTART);    // Depuracion por USB
					}
				}
			break;

//			case 12:	//Envio comando
//				PutStrOnTx(&TXUSART1,CIPSEND); //Envio comando AT para ver si responde el ESP
//				PutStrOnTx(&TXUSB,CIPSEND);    // Depuracion por USB
//				RXUSART1->state = 13;
//			break;
//
//			case 13: //Leo respuesta
//				DecodeANS(RXUSART1,ANS_CIPSEND);
//				PutStrOnTx(&TXUSB,ANS_CIPSEND);    // Depuracion por USB
//				RXUSART1->state = 14;
//			break;
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
/***************** Contadores para control del flujo de tiempo ********************/
/*
 * Instancia cada 1ms
 *
 */
	if(htim->Instance == TIM4){
		Count100ms--;

		if(!Count100ms){
			Count100ms = 100;
			On100ms = 1;

			Count3000ms--;
			if(!Count3000ms){
				On3000ms = 1;
			}

			Count500ms--;
			if(!Count500ms){
				On500ms = 1;
			}
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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	RXUSART1.iw++;
	HAL_UART_Receive_IT(&huart1, &rxUSART1Buff[RXUSART1.iw], 1);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	volatile uint8_t c;
/********************* Media movil de las 3 primeras lecturas **********************/
	if(FirtScan){
		switch(iAdc){
			case 1:
				for(c=0;c<8;c++){
					bufADC[1][c]=(bufADC[0][c]+bufADC[1][c]) / 2 ;
				}
			break;
			case 2:
				for(c=0;c<8;c++){
					bufADC[2][c]=(bufADC[0][c]+bufADC[1][c]+bufADC[2][c]) / 3;
				}
				FirtScan = 0;
			break;

			default:
			break;
		}
	}

/***********************************************************************************/
/********************* Media movil de las siguientes lecturas **********************/
	else{
		if(iAdc >= 0 && iAdc <= 1){
			// Cruce por 0 y 1 donde se toman dos valores anteriores para hacer la media
			switch(iAdc){
				case 0:
					for(c = 0; c<8; c++){
						bufADC[0][c]=( bufADC[0][c]+bufADC[31][c]+bufADC[30][c]) / 3 ;
					}
				break;
				case 1:
					for(c = 0; c<8; c++){
						bufADC[1][c]=(bufADC[0][c]+bufADC[1][c]+bufADC[31][c]) / 3;
					}
				break;
			}
		}
		else{
			// Cálculo comprendido entre bufADC[2] y bufADC[31]
			for(c = 0; c<8; c++){
				bufADC[iAdc][c] = (bufADC[iAdc-2][c] + bufADC[iAdc-1][c] + bufADC[iAdc][c])/3;
			}
		}
	}
/***********************************************************************************/
	// Incrento indice del buffer
	iAdc++;
	if(iAdc == 32){
		iAdc = 0;
	}
}

void MyCallBackOnUSBData(uint8_t *buf, uint32_t len){
	for(uint32_t i=0; i<len; i++){
		rxUSBBuff[RXUSB.iw++] = buf[i];
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
                    }
                }
                break;

            default:
                RX->header = 0;
                break;
        }
        RX->ir &= RX->maskSize;
        RX->ir ++;
        RX->ir &= RX->maskSize;                                                // Enmascaro el indice de lectura - Cuando llega a 63 se pone en 0
    }
}

void DecodeCmd(_Rx *RX)
{
    RX->ISCMD = 0;

    switch(RX->Buff[RX->iData]){

    	/*
		 * Comando para enviar datos IR
		 */
		case 0xA0:                                                              //Sensores analogicos
			PutHeaderOnTx(&TXUSB, 0xA0, 17);
			if(iAdc >= 2 && iAdc <= 31){
				PutBuffOnTx(&TXUSB, (uint8_t *)&bufADC[iAdc-2], 16);
			}
			PutcksOnTx(&TXUSB);
		break;

    	/*
         * Comando para enviar alive
         */
        case 0xF0:                                                              //Alive
            PutHeaderOnTx(&TXUSB, 0xF0, 2);
            PutByteOnTx(&TXUSB, 0x0D);
            PutcksOnTx(&TXUSB);
            break;
        /*
         * Comando para enviar aviso de error
         */
        default:
            PutHeaderOnTx(&TXUSB, 0xFF, 1);
            PutcksOnTx(&TXUSB);
            break;
    }
}

uint8_t DecodeANS(_Rx *RX, const char *str,uint8_t strlen){
	uint8_t aux = RX->ir;
	for(uint8_t i=0; i<strlen; i++){
		if(str[i]!=RX->Buff[RX->ir]){
			RX->ir = aux;
			return 0;
		}
		if(RX->Buff[RX->ir]=='K'){
			return 1;
		}
		RX->ir++;
	}
	return 1;
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

void PutHeaderOnTx(_Tx *TX, uint8_t id, uint8_t lcmd)
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
    TX->length = lcmd;
    TX->Buff[TX->iw++] = lcmd + 1;
    TX->iw &= TX->maskSize;
    TX->Buff[TX->iw++] = ':';
    TX->iw &= TX->maskSize;
    TX->Buff[TX->iw++] = id;
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

void PutStrOnTx(_Tx *TX, const char *str)
{
    uint8_t i = 0;

    while(str[i]) {
        TX->Buff[TX->iw++] = str[i++];
        TX->iw &= TX->maskSize;
    }
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
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);

  //Interrupcion para recibir datos desde ESP8266
  HAL_UART_Receive_IT(&huart1,rxUSART1Buff,1);

/***********************************************************************************/
/************************** Inicializacion de contadores ***************************/
/***********************************************************************************/
  Count100ms = 100;
  Count200ms = 0;
  Count3000ms = 0;
  Count500ms = 0;
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
/***********************************************************************************/
/***********************************************************************************/
/***********************************************************************************/

/***********************************************************************************/
/************************ Inicializacion de  ADC **************************/
/***********************************************************************************/
  iAdc = 0;
  FirtScan=1;
/***********************************************************************************/
/***********************************************************************************/
/***********************************************************************************/


/***********************************************************************************/
/***************** Inicializacion de  error cuadratico ********************/
/***********************************************************************************/
  posMINCenter = 0;
  posMINRight = 0;
  posMINLeft = 0;
  sensorValue = 0;
  error = 0;
/***********************************************************************************/
/***********************************************************************************/
/***********************************************************************************/

/***********************************************************************************/
/************************* Inicialización de variables PID *************************/
/***********************************************************************************/
  Kp = 0;
  Ki = 0;
  Kd = 0;
  Proporcional = 0;
  Integral = 0;
  Derivativo = 0;
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
	  if(On100ms){
		  On100ms = 0;
		  HAL_GPIO_TogglePin(LED13_GPIO_Port, LED13_Pin);
	  }

	  //Recepcion por USB
	  if(RXUSB.iw != RXUSB.ir) {
		  DecodeHeader((_Rx *)&RXUSB);
	  }

	  if(RXUSB.ISCMD) {
		  DecodeCmd((_Rx *)&RXUSB);
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
	  //Recepcion por ESP8266
	  	  if(RXUSART1.iw != RXUSART1.ir) {
	  		  DecodeHeader((_Rx *)&RXUSART1);
	  	  }

	  	  if(RXUSART1.ISCMD) {
	  		  DecodeCmd((_Rx *)&RXUSART1);
	  	  }

	  //Transmision por ESP8266
	  if(TXUSART1.iw != TXUSART1.ir) {
		  if(huart1.Instance->SR){
			  huart1.Instance->DR = txUSART1Buff[TXUSART1.ir++];
			  TXUSART1.ir=TXUSART1.iw;
		  }
		  else{
			  huart1.Instance->DR = txUSART1Buff[TXUSART1.ir++];
		  }
	  }
//	  if(TXUSART1.iw != TXUSART1.ir) {
//	  		  if(TXUSART1.iw > TXUSART1.ir){
//	  			if(HAL_OK == HAL_UART_Transmit_IT(&huart1, txUSART1Buff, (TXUSART1.iw-TXUSART1.ir))){
//	  					TXUSART1.ir=TXUSART1.iw;
//	  		  }
//	  		  else{
//	  			  w.u16[0] = (TXUSART1.maskBuf+1)-TXUSART1.ir;
//	  			if(HAL_OK == HAL_UART_Transmit_IT(&huart1, txUSART1Buff, w.u16[0]))
//	  				TXUSART1.ir = 0;
//	  		  }
//	  		TXUSART1.ir &= TXUSART1.maskSize;
//	  	  }
//	  }
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
  HAL_GPIO_WritePin(RESET_ESP_GPIO_Port, RESET_ESP_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pin : RESET_ESP_Pin */
  GPIO_InitStruct.Pin = RESET_ESP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RESET_ESP_GPIO_Port, &GPIO_InitStruct);

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
