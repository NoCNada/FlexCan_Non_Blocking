/*
 * Copyright 2016-2023 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file    FlexCanNonBlocking.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
/* TODO: insert other include files here. */
void CAN0_CAN_ORED_MB_IRQHANDLER(void);

#define BOARD_LED_GPIO     BOARD_LED_RED_GPIO
#define BOARD_LED_GPIO_PIN BOARD_LED_RED_PIN
#define RX_MESSAGE_BUFFER_NUM (0)
#define TX_MESSAGE_BUFFER_NUM (1)
#define RX_ID 0x10
#define TX_ID 0x123

/* TODO: insert other definitions and declarations here. */

void PIT_CHANNEL_0_IRQHANDLER(void);
void CAN0_CAN_ORED_MB_IRQHANDLER(void);
void rxCompleteCallback(CAN_Type *base, flexcan_handle_t *handle, status_t status, uint32_t result, void *userData);
void txCompleteCallback(CAN_Type *base, flexcan_handle_t *handle, status_t status, uint32_t result, void *userData);
static void flexcan_callback(CAN_Type *base, flexcan_handle_t *handle, status_t status, uint32_t result, void *userData);


flexcan_handle_t rxHandle, txHandle, flexcanHandle;
flexcan_frame_t txFrame, rxFrame;
flexcan_mb_transfer_t txXfer, rxXfer;
//flexcan_frame_t frame;
flexcan_rx_mb_config_t mbConfig;
uint8_t rxComplete = 0, txComplete = 0, counter = 0, txFlag = 0, counterTime = 50;
uint16_t waitTX = 1000, waitRX = 40;

/*
 * @brief   Application entry point.
 */
int main(void) {

    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
    //FLEXCAN_EnableMbInterrupts(CAN0, 1 << 0);

    LED_RED_INIT(0);
    LED_GREEN_INIT(0);
    LED_BLUE_INIT(0);

#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif

    // config TX CAN
    txFrame.format = (uint8_t)kFLEXCAN_FrameFormatStandard;
    txFrame.id = FLEXCAN_ID_STD(TX_ID);
    txFrame.type = (uint8_t)kFLEXCAN_FrameTypeData;
    txFrame.length = (uint8_t)2; // DLC
    txFrame.dataByte0 = (uint8_t)30;
    txFrame.dataByte1 = (uint8_t)5;
    txFrame.length = 2;
    txXfer.mbIdx = TX_MESSAGE_BUFFER_NUM;
    txXfer.frame = &txFrame;

    // Config RX CAN
   // rxFrame.format = (uint8_t)kFLEXCAN_FrameFormatStandard;
   // rxFrame.id = FLEXCAN_ID_STD(RX_ID);
   // rxFrame.type = (uint8_t)kFLEXCAN_FrameTypeData;
    mbConfig.format = kFLEXCAN_FrameFormatStandard;
    mbConfig.type   = kFLEXCAN_FrameTypeData;
    mbConfig.id = FLEXCAN_ID_STD(0x10);
    FLEXCAN_SetRxMbConfig(CAN0, 0, &mbConfig, true);
    rxXfer.mbIdx = RX_MESSAGE_BUFFER_NUM;
    rxXfer.frame = &rxFrame;

    //Init

    //FLEXCAN_TransferCreateHandle(CAN0, &flexcanHandle, flexcan_callback, NULL);
    FLEXCAN_TransferCreateHandle(CAN0, &flexcanHandle, flexcan_callback, NULL);

    PRINTF("Hello World\n");
    //inicializo nonBlocking

    FLEXCAN_TransferSendNonBlocking(CAN0, &flexcanHandle, &txXfer);
    FLEXCAN_TransferReceiveNonBlocking(CAN0, &flexcanHandle, &rxXfer);
    GPIO_PortToggle(BOARD_LED_GPIO, 1u << BOARD_LED_GPIO_PIN);
   // FLEXCAN_TransferAbortReceive(CAN0, &flexcanHandle, RX_MESSAGE_BUFFER_NUM);
    /* Force the counter to be placed into memory. */
    volatile static int i = 0 ;
    /* Enter an infinite loop, just incrementing a counter. */
    while(1) {
        i++ ;
        FLEXCAN_TransferReceiveNonBlocking(CAN0, &flexcanHandle, &rxXfer);
        //FLEXCAN_TransferReceiveNonBlocking(CAN0, &flexcanHandle, &rxXfer);
         if(txComplete){
        	 waitTX = 1000;
        	 //txFlag = 1; // para mandar otro dato en 1segundo
        	 txComplete = 0;
        	 //FLEXCAN_TransferAbortSend(CAN0, &flexcanHandle, TX_MESSAGE_BUFFER_NUM);
         }

         if(rxComplete){
        	 counterTime=rxFrame.dataByte0*10;
        	 //txFlag = 1;
        	 rxComplete = 0;
         }

         if(!counter){
         	GPIO_PortToggle(BOARD_LED_GPIO, 1u << BOARD_LED_GPIO_PIN);
         	LED_BLUE_TOGGLE();
         	LED_GREEN_TOGGLE();
         	counter=counterTime;
         }

/*         if(txFlag && !waitTX){
        	 txFrame.dataByte0 = (uint8_t)((counterTime+10)/10);
        	 txFrame.dataByte1 = (uint8_t)5;
        	 FLEXCAN_TransferSendNonBlocking(CAN0, &flexcanHandle, &txXfer);
        	 waitTX = 1000;
         }*/
        /* 'Dummy' NOP to allow source level single stepping of
            tight while() loop */
        __asm volatile ("nop");
    }
    return 0 ;
}




static void flexcan_callback(CAN_Type *base, flexcan_handle_t *handle, status_t status, uint32_t result, void *userData){
	switch (status){
	/* Process FlexCAN Rx event. */
	case kStatus_FLEXCAN_RxIdle:
		//PRINTF("prueba \n");
		LED_BLUE_TOGGLE();
		if (RX_MESSAGE_BUFFER_NUM == result)
		{
			rxComplete = 1;
		}
		break;
	case kStatus_FLEXCAN_RxOverflow:
		LED_BLUE_TOGGLE();
		if (RX_MESSAGE_BUFFER_NUM == result)
		{
			rxComplete = 1;
		}
		break;
		/* Process FlexCAN Tx event. */
	case kStatus_FLEXCAN_TxIdle:
		LED_GREEN_TOGGLE();
		if (TX_MESSAGE_BUFFER_NUM == result)
		{
			//txComplete = 1;
		}
		break;
	case kStatus_FLEXCAN_TxBusy:
		LED_GREEN_TOGGLE();
		if (TX_MESSAGE_BUFFER_NUM == result)
		{
			//retry
		}
		break;

	default:
		break;
	}
}


/*

// Callback function for received messages
void rxCompleteCallback(CAN_Type *base, flexcan_handle_t *handle, status_t status, uint32_t result, void *userData){
// Update the data length code to indicate that a message has been received
	status;
	rxFrame.length = result;
	rxComplete = 1;
}


void txCompleteCallback(CAN_Type *base, flexcan_handle_t *handle, status_t status, uint32_t result, void *userData) {
  // Clear the data length code to mark the transmit buffer as available
	status;
	//txFrame.length = 0;
	txFlag = 0;
  //txFrame.length
}

*/


/* PIT0_IRQn interrupt handler */
void PIT_CHANNEL_0_IRQHANDLER(void) {
	uint32_t intStatus;
	/* Reading all interrupt flags of status register */
	intStatus = PIT_GetStatusFlags(PIT_PERIPHERAL, PIT_CHANNEL_0);
	PIT_ClearStatusFlags(PIT_PERIPHERAL, PIT_CHANNEL_0, intStatus);

	/* Place your code here */
	if(counter)
		counter--;
	if(waitTX)
		waitTX--;
	if(waitRX)
			waitRX--;
	/* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F
     Store immediate overlapping exception return operation might vector to incorrect interrupt. */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
	__DSB();
#endif
}
