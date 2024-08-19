
/**
  ******************************************************************************
  * File Name          :  app_nfc7.c
  * Description        : This file provides code for the configuration
  *                      of the STMicroelectronics.X-CUBE-NFC7.1.0.1 instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "app_nfc7.h"
#include "main.h"

/* Includes ------------------------------------------------------------------*/
#include "common.h"
#include "tagtype5_wrapper.h"
#include "lib_NDEF_URI.h"

#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "task.h"
#include "FreeRTOS.h"

/** @defgroup ST25_Nucleo
  * @{
  */

/** @defgroup Main
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Global variables ----------------------------------------------------------*/
sURI_Info URI;

extern sCCFileInfo CCFileStruct;

/* Private functions ---------------------------------------------------------*/
void MX_NFC7_NDEF_URI_Init(void);
void MX_NFC7_NDEF_URI_Process(void);

void MX_NFC7_Init(void)
{
  /* USER CODE BEGIN SV */

	NFC07A1_LED_Init(GREEN_LED );
	NFC07A1_LED_Init(BLUE_LED );
	NFC07A1_LED_Init(YELLOW_LED );

	NFC07A1_LED_On( GREEN_LED );
	osDelay( 30 );
	NFC07A1_LED_On( BLUE_LED );
	osDelay( 30 );
	NFC07A1_LED_On( YELLOW_LED );
	osDelay( 30 );

	while( NFC07A1_NFCTAG_Init(NFC07A1_NFCTAG_INSTANCE) != NFCTAG_OK );

	NFC07A1_NFCTAG_SetRFDisable_Dyn(0);

	NFC07A1_GPO_Init();

	NFC07A1_NFCTAG_ConfigIT(0,0x82);

	if( NfcType5_NDEFDetection( ) != NDEF_OK )
	{
	  CCFileStruct.MagicNumber = NFCT5_MAGICNUMBER_E1_CCFILE;
	  CCFileStruct.Version = NFCT5_VERSION_V1_0;
	  CCFileStruct.MemorySize = ( ST25DVXXKC_MAX_SIZE / 8 ) & 0xFF;
	  CCFileStruct.TT5Tag = 0x01;
	  /* Init of the Type Tag 5 component (M24LR) */

	  NfcTag_SelectProtocol(NFCTAG_TYPE5);

	  while( NfcType5_TT5Init( ) != NFCTAG_OK );
	}

	NDEF_ClearNDEF();

	NFC07A1_LED_Off( GREEN_LED );
	osDelay( 30 );
	NFC07A1_LED_Off( BLUE_LED );
	osDelay( 30 );
	NFC07A1_LED_Off( YELLOW_LED );
	osDelay( 30 );

	NFC07A1_LED_On( BLUE_LED );
	osDelay( 30 );

  /* USER CODE END SV */

  /* USER CODE BEGIN NFC7_Library_Init_PreTreatment */


	#if 0
  /* USER CODE END NFC7_Library_Init_PreTreatment */

  /* Initialize the peripherals and the NFC7 components */
  MX_NFC7_NDEF_URI_Init();

  /* USER CODE BEGIN SV */

	#endif

  /* USER CODE END SV */

  /* USER CODE BEGIN NFC7_Library_Init_PostTreatment */

  /* USER CODE END NFC7_Library_Init_PostTreatment */
}
/*
 * LM background task
 */
void MX_NFC7_Process(void)
{
  /* USER CODE BEGIN NFC7_Library_Process */

  /* USER CODE END NFC7_Library_Process */

  MX_NFC7_NDEF_URI_Process();

}

void MX_NFC7_NDEF_URI_Init(void)
{
	  /******************************************************************************/
  /* Configuration of X-NUCLEO-NFC02A1                                          */
  /******************************************************************************/
  /* Init of the Leds on X-NUCLEO-NFC07A1 board */
  NFC07A1_LED_Init(GREEN_LED );
  NFC07A1_LED_Init(BLUE_LED );
  NFC07A1_LED_Init(YELLOW_LED );
  NFC07A1_LED_On( GREEN_LED );
  osDelay( 30 );
  NFC07A1_LED_On( BLUE_LED );
  osDelay( 30 );
  NFC07A1_LED_On( YELLOW_LED );
  osDelay( 30 );

  /* Init ST25DVXXKC driver */
  while( NFC07A1_NFCTAG_Init(NFC07A1_NFCTAG_INSTANCE) != NFCTAG_OK );

  /* Reset Mailbox enable to allow write to EEPROM */
  NFC07A1_NFCTAG_ResetMBEN_Dyn(NFC07A1_NFCTAG_INSTANCE);

  NfcTag_SelectProtocol(NFCTAG_TYPE5);

  /* Check if no NDEF detected, init mem in Tag Type 5 */
  if( NfcType5_NDEFDetection( ) != NDEF_OK )
  {
    CCFileStruct.MagicNumber = NFCT5_MAGICNUMBER_E1_CCFILE;
    CCFileStruct.Version = NFCT5_VERSION_V1_0;
    CCFileStruct.MemorySize = ( ST25DVXXKC_MAX_SIZE / 8 ) & 0xFF;
    CCFileStruct.TT5Tag = 0x05;
    /* Init of the Type Tag 5 component (M24LR) */
    while( NfcType5_TT5Init( ) != NFCTAG_OK );
  }

  /* Init done */
  NFC07A1_LED_Off( GREEN_LED );
  osDelay( 30 );
  NFC07A1_LED_Off( BLUE_LED );
  osDelay( 30 );
  NFC07A1_LED_Off( YELLOW_LED );
  osDelay( 30 );

  /* Prepare URI NDEF message content */
  strcpy( URI.protocol,URI_ID_0x01_STRING );
  //strcpy( URI.URI_Message,"st.com/st25-demo" );
  strcpy( URI.URI_Message,"fastlogic.pl" );
  strcpy( URI.Information,"\0" );

  /* Write NDEF to EEPROM */
  osDelay(5);
  while( NDEF_WriteURI( &URI ) != NDEF_OK );

  /* Set the LED3 on to indicate Programing done */
  NFC07A1_LED_On( YELLOW_LED );

}

/**
  * @brief  Process of the NDEF_URI application
  * @retval None
  */
void MX_NFC7_NDEF_URI_Process(void)
{

}

#ifdef __cplusplus
}
#endif

