/*
 * Copyright 2016-2020 NXP
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
 * @file    Akcelerometer.cpp
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL25Z4.h"
#include "fsl_debug_console.h"
#include "fsl_i2c.h"
#include "MMA8451Q.h"
#include "fsl_pit.h"
#include <math.h>
#include "fsl_uart.h"
#include "fsl_common.h"
#include "fsl_lpsci.h"
#define alpha 0.09

float x,y,z = 0;

float acc_x, acc_y,acc_z = 0;
MMA8451Q *acc_pt;
/* TODO: insert other include files here. */

/* TODO: insert other definitions and declarations here. */

/*
 * @brief   Application entry point.
 */


volatile bool pitIsrFlag = false;


extern "C" void PIT_IRQHandler() {
    PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, kPIT_TimerFlag);
    pitIsrFlag = true;
}

void collectData() {
	float res[3];
	acc_pt->getAccAllAxis(res);

	res[0] = res[0]/4096;
	res[1] = res[1]/4096;
	res[2] = res[2]/4096;


	x = (1 - alpha) * x + alpha * res[0];
	y = (1 - alpha) * y + alpha * res[1];
	z = (1 - alpha) * z + alpha * res[2];

	acc_x = res[0] - x;
	acc_y = res[1] - y;
	acc_z = res[2] - z;

	double result = acos(res[0]/9.81) * 180 / 3.14;

	PRINTF("%f %f %f %f\r\n",x,y,z,result);
	pitIsrFlag = false;
}


int CRC = 0;

int crc[] = { 0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
	    157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
	    35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
	    190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
	    70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
	    219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
	    101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
	    248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
	    140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
	    17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
	    175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
	    50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
	    202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
	    87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
	    233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
	    116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53 };


int main(void) {

  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif

    MMA8451Q acc(I2C0,0x1D);
    acc_pt = &acc;


    pit_config_t pitConfig;

    PIT_GetDefaultConfig(&pitConfig);

    /* Init pit module */
    PIT_Init(PIT, &pitConfig);
    /* Set timer period for channel 0 */
    //50U => 1/20*1000
    PIT_SetTimerPeriod(PIT, kPIT_Chnl_0,
    		MSEC_TO_COUNT(50U, CLOCK_GetBusClkFreq()));

    /* Enable timer interrupts for channel 0 */
    PIT_EnableInterrupts(PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable);
    PIT_StartTimer(PIT, kPIT_Chnl_0);
    /* Enable at the NVIC */
    EnableIRQ (PIT_IRQn);

	/* Inicializacia UART0 (LPSCI)*/
	CLOCK_SetLpsci0Clock(0x1U); // Zapnutie hodin tu alebo cez GUI clock wizzard

	lpsci_config_t user_config;
	LPSCI_GetDefaultConfig(&user_config);
	user_config.baudRate_Bps = 9600U;
	user_config.enableRx = true;
	user_config.enableTx = true;
	LPSCI_Init(UART0, &user_config, CLOCK_GetPllFllSelClkFreq());
	PRINTF("FFFFFF %f\n",50.5);

	char message[2] = {0x00,0xC4};

	for(int i = 0; i < 2; i++) {
		CRC = crc[CRC ^ message[i]];

	}
	PRINTF("CRC %d, %x",CRC,CRC);
    while(1) {
    	if(pitIsrFlag == true) {
    		collectData();
    	}
    }
    return 0 ;
}
