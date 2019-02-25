/*
 * init_utils.h
 *
 *  Created on: 19.02.2019
 *      Author: nickf
 */

#ifndef SRC_INIT_UTILS_H_
#define SRC_INIT_UTILS_H_

// address definitions for devices inside bsp
#include "xparameters.h" 		// [pl]
#include "xparameters_ps.h"		// [ps]
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

#define INIT_CONFIG_CHECK(msg, device) if (!cfgPtr) {xil_printf(msg,device); return XST_FAILURE;}
#define INIT_STATUS_CHECK(msg, state) if (state != XST_SUCCESS) {xil_printf(msg,state); return XST_FAILURE;}

/// ####################################################################################

/// this is my library
struct InitUtils {
	/// hold function pointers
    int (*initDma) (XAxiDma* dma_inst, int dma_device_id);
    int (*initGpio) (XGpio* gpio_inst, int gpio_device_id, unsigned channel, u32 direction);
    int (*initIntc) (XScuGic* intc_inst, int intc_device_id);
    int (*intcAddInterruptHandler) (Xil_InterruptHandler function, void* obj , XScuGic* intc_inst, int intc_device_id, int interrupt_id, int priority);
};
/// this is my library api handler obj
extern const struct InitUtils initUtils;



#endif /* SRC_INIT_UTILS_H_ */
