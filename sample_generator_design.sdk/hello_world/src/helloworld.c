/******************************************************************************
*
* Copyright (C) 2009 - 2014 Xilinx, Inc.  All rights reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* Use of the Software is limited solely to applications:
* (a) running on a Xilinx device, or
* (b) that interact with a Xilinx device through a bus or interconnect.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
* XILINX  BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
* OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
* Except as contained in this notice, the name of the Xilinx shall not be used
* in advertising or otherwise to promote the sale, use or other dealings in
* this Software without prior written authorization from Xilinx.
*
******************************************************************************/

/*
 * helloworld.c: simple test application
 *
 * This application configures UART 16550 to baud rate 9600.
 * PS7 UART (Zynq) is not initialized by this application, since
 * bootrom/bsp configures it to baud rate 115200
 *
 * ------------------------------------------------
 * | UART TYPE   BAUD RATE                        |
 * ------------------------------------------------
 *   uartns550   9600
 *   uartlite    Configurable only in HW design
 *   ps7_uart    115200 (configured by bootrom/bsp)
 */


//#define DEBUG

// ****************************************************
#include <stdio.h>
#include "sleep.h"
// address definitions for devices inside bsp
#include "xparameters.h" 		// [pl]
#include "xparameters_ps.h"		// [ps]
#include "xdebug.h"
// specific project platform definitions / initializations
#include "platform.h"
#include "ps7_init.h"
// xilinx utility functions
#include "xil_printf.h"			// uart io
#include "xil_io.h"				// write/read from address
// pl device drivers
#include "xgpio.h"				// gpio driver
#include "xscugic.h"			// axi interrupt control driver
#include "xscutimer.h"			// timer driver
#include "xaxidma.h"			// axi dma driver
// ****************************************************
// kikass stuff
#include "init_utils.h"
/// #################################################################################

#define DMA_RESET_TIMEOUT_COUNTER 10000

static XAxiDma AxiDma;
static XScuGic Intc;
static XGpio AxiLeds;

static XGpio AxiSampleGenFrameSize;
static XGpio AxiSampleGenEnable;

static int globalError = 0;
static int txDone = 0;
static int rxDone = 0;

#define COUNTER_FRAME_SIZE 0x0f

static void dma_s2mm_isr(void *Callback);


void dma_s2mm_isr(void* CallbackRef)
{
	uint32_t irq_status;
	XAxiDma* dma_inst = (XAxiDma*)CallbackRef;

	// Disable interrupts
	XAxiDma_IntrDisable(dma_inst, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DMA_TO_DEVICE);
	XAxiDma_IntrDisable(dma_inst, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DEVICE_TO_DMA);

	// Read pending interrupts
	irq_status = XAxiDma_IntrGetIrq(dma_inst, XAXIDMA_DEVICE_TO_DMA);
	xil_printf("INT.[IOC:%d][DEL:%d][ERR:%d]\r\n", irq_status & XAXIDMA_IRQ_ALL_MASK,
			irq_status & XAXIDMA_IRQ_DELAY_MASK, irq_status & XAXIDMA_IRQ_ERROR_MASK);

	// Acknowledge pending interrupts
	XAxiDma_IntrAckIrq(dma_inst, irq_status, XAXIDMA_DEVICE_TO_DMA);

	// If no interrupt is asserted, we do not do anything
	if (!(irq_status & XAXIDMA_IRQ_ALL_MASK)) return;

	// If error interrupt is asserted, raise error flag, reset the
	// hardware to recover from the error, and return with no further
	// processing.
	if ((irq_status & XAXIDMA_IRQ_DELAY_MASK))
	{
		xil_printf("IRQ_DELAY-FLAG set!\n\r");
	}

	if ((irq_status & XAXIDMA_IRQ_ERROR_MASK))
	{
		xil_printf("IRQ_ERROR-FLAG set!\n\r");
		globalError = 1;

		// Reset should never fail for transmit channel
		int time_out = 0;
		XAxiDma_Reset(dma_inst);
		while (time_out < DMA_RESET_TIMEOUT_COUNTER)
		{
			if (XAxiDma_ResetIsDone(dma_inst))
			{
				break;
			}
			time_out += 1;
		}
	}

	// Re-enable interrupts
	XAxiDma_IntrEnable(dma_inst, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DMA_TO_DEVICE);
	XAxiDma_IntrEnable(dma_inst, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DEVICE_TO_DMA);
	//XAxiDma_IntrEnable(dma_inst, (XAXIDMA_IRQ_IOC_MASK | XAXIDMA_IRQ_ERROR_MASK), XAXIDMA_DMA_TO_DEVICE);
	//XAxiDma_IntrEnable(dma_inst, (XAXIDMA_IRQ_IOC_MASK | XAXIDMA_IRQ_ERROR_MASK), XAXIDMA_DEVICE_TO_DMA);

	if(globalError)
		return;

	// Completion interrupt asserted
	if (irq_status & XAXIDMA_IRQ_IOC_MASK)
		txDone = 1;
}

int enableSampleGenerator()
{
	XGpio_DiscreteWrite(&AxiSampleGenEnable, 1, 0x01);
	return XST_SUCCESS;
}
int disableSampleGenerator()
{
	XGpio_DiscreteWrite(&AxiSampleGenEnable, 1, 0x00);
	return XST_SUCCESS;
}
int initSampleGenerator(int frameSize)
{
	int status;
	status = initUtils.initGpio(&AxiSampleGenFrameSize, XPAR_AXI_GPIO_0_8BIT_FRAMESIZE_DEVICE_ID, 1, 0x00000000);
	if (status != XST_SUCCESS){ return status; }
	status = initUtils.initGpio(&AxiSampleGenEnable, XPAR_AXI_GPIO_1_1BIT_EN_DEVICE_ID, 1, 0x00000000);
	if (status != XST_SUCCESS){ return status; }
	XGpio_DiscreteWrite(&AxiSampleGenFrameSize, 1, frameSize);
	disableSampleGenerator();
	return XST_SUCCESS;
}


int init()
{

	int status;

	// initialize sample generator
	xil_printf("Initializing Sample Generator...\n\r");
	status = initSampleGenerator(COUNTER_FRAME_SIZE);
	if (status != XST_SUCCESS){ xil_printf("Failed! \n\r");return status; }
	// initialize debug leds gpio
    xil_printf("Initializing Debug Leds Gpio ...\n\r");
	status = initUtils.initGpio(&AxiLeds, XPAR_AXI_GPIO_2_8BIT_LEDS_DEVICE_ID, 1, 0x00000000);
	if (status != XST_SUCCESS){ xil_printf("Failed! \n\r");return status; }
	// initialize dma controller
	xil_printf("Initializing DMA Controller...\n\r");
	status = initUtils.initDma(&AxiDma, XPAR_AXI_DMA_0_DEVICE_ID);
	if (status != XST_SUCCESS){ xil_printf("Failed! \n\r");return status; }
	// initialize interrupt controller
    xil_printf("Initializing Interrupt Controller...\n\r");
	status = initUtils.initIntc(&Intc, XPAR_SCUGIC_SINGLE_DEVICE_ID);
	if (status != XST_SUCCESS){ xil_printf("Failed! \n\r");return status; }
	/// initialize interrupt service routines
	// for dma
	initUtils.intcAddInterruptHandler(	(Xil_InterruptHandler)dma_s2mm_isr, &AxiDma, &Intc, XPAR_SCUGIC_SINGLE_DEVICE_ID,
										XPAR_FABRIC_AXI_DMA_0_S2MM_INTROUT_INTR, 0xA0);

	return status;
}


#define N 8
unsigned ror(unsigned int x, unsigned int bits){
	const unsigned int mask = (N - 1);
	bits &= mask;
	return (x>>bits) | (x<<( (-bits)&mask ));
}
unsigned rol(unsigned int x, unsigned int bits){
	const unsigned int mask = (N - 1);
	bits &= mask;
	return (x<<bits) | (x>>( (-bits)&mask ));
}



int triggerSampleGen2DmaTransfer(unsigned int addr, unsigned int len)	// word_count(for example 15) * word_size(usually 32 bit = 4 bytes)
{
	int status;
	u32 delay = 100 * 1000; //ms
	u32 c = 7;
	// ??? flush cache ???  further research needed
	Xil_DCacheFlushRange(addr, len);

	// start dma transfer
	xil_printf("Starting DMA ...\n\r");
	status = XAxiDma_SimpleTransfer(&AxiDma, addr, len, XAXIDMA_DEVICE_TO_DMA);
	if (status != XST_SUCCESS){xil_printf("ERROR! Failed to kick off S2MM transfer! [%d]\n\r", status);return XST_FAILURE;}
	xil_printf("Waiting ...\n\r");

	// the sample generator (axi_stremer) module will wait for enable to kick off a waiting routine
	// after a delay, it will force TVALID high and wait for TREADY (from dma ctrl), it is then ready so transmit data
	// it will now proceed to increment a counter up to (frame_size parameter (0 to 32bit)
	// after each overflow, it will create a tlast (axi_stream), which will tell the dma that a frame is finished
	// if tready should go LOW while counting (ex: dma ctrl needs more time cause of MEM is busy), the module will wait until tready is HIGH again
	enableSampleGenerator();
	// Wait for transfer to complete
	while (!txDone && !globalError && !rxDone)
	{
		// KNIGHT RIDERzZ
		XGpio_DiscreteWrite(&AxiLeds, 1, c);
		usleep(delay);
		c = rol(c,1);
	}
	// disable out sample generator
	disableSampleGenerator();
	// Check DMA for errors
	if (globalError)
	{
		xil_printf("ERROR! AXI DMA returned an error during the S2MM transfer.\n\r");
		status = XST_FAILURE;
	}
	//reset our global signaling variables
	globalError = 0; txDone = 0; rxDone = 0;
	return status;
}

int work()
{
	int status;
	unsigned int addr;
	unsigned int byteCount = COUNTER_FRAME_SIZE * 4;	// word_count(for example 15) * word_size(usually 32 bit = 4 bytes)
	//only a specific address range is mapped for HP0 dma access, see:
	// https://www.xilinx.com/support/documentation/user_guides/ug585-Zynq-7000-TRM.pdf
	addr = 0x0a000300;
	// trigger data transfer
	status = triggerSampleGen2DmaTransfer(addr, byteCount);
	// trigger data transfer
	addr += byteCount;
	status = triggerSampleGen2DmaTransfer(addr, byteCount);
	// ...
    return status;
}

int main()
{
	int status;
    // initiate zynq
    init_platform();
    // enable level shifters and release reset signals
    ps7_post_config();
    xil_printf("\n\r===============================\n\r");
    status = init();
    if (status == XST_SUCCESS)
    {
    	xil_printf("Ima ZYNQ boiiiii <3\n\r");
    	status = work();
        xil_printf("Done.\n\r");
    }

    cleanup_platform();
    return status;
}
