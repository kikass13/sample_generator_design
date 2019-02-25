#include "init_utils.h"

int initDma(XAxiDma* dma_inst, int dma_device_id)
{
	int status;
	XAxiDma_Config* cfgPtr;
	/// grab config
	cfgPtr = XAxiDma_LookupConfig(dma_device_id);
	INIT_CONFIG_CHECK("ERROR! No hardware configuration found for DMA Controller with device id %d.\r\n", dma_device_id);
	/// initialize dma device
	status = XAxiDma_CfgInitialize(dma_inst, cfgPtr);
	INIT_STATUS_CHECK("ERROR! Initialization of DMA Controller failed %d\r\n", status);
	/// is this scatter gather?
	status = XAxiDma_HasSg(dma_inst);
	INIT_STATUS_CHECK("DMA Device configured as SG mode \r\n", status);
	// Disable all interrupts
	XAxiDma_IntrDisable(dma_inst, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DMA_TO_DEVICE);
	XAxiDma_IntrDisable(dma_inst, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DEVICE_TO_DMA);
	// Enable specific interrupts
	XAxiDma_IntrEnable(dma_inst, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DMA_TO_DEVICE);
	XAxiDma_IntrEnable(dma_inst, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DEVICE_TO_DMA);
	//XAxiDma_IntrEnable(dma_inst, (XAXIDMA_IRQ_IOC_MASK | XAXIDMA_IRQ_ERROR_MASK), XAXIDMA_DMA_TO_DEVICE);
	//XAxiDma_IntrEnable(dma_inst, (XAXIDMA_IRQ_IOC_MASK | XAXIDMA_IRQ_ERROR_MASK), XAXIDMA_DEVICE_TO_DMA);
	return XST_SUCCESS;
}

int initGpio (XGpio* gpio_inst, int gpio_device_id, unsigned channel, u32 direction)
{
	int status;
	/* Initialize the GPIO driver */
	status = XGpio_Initialize(gpio_inst, gpio_device_id);
	INIT_STATUS_CHECK("ERROR! Initialization of GPIO failed %d\r\n", status);
	/* Set the direction for all signals as inputs except the LED output */
	XGpio_SetDataDirection(gpio_inst, channel, direction);
	return XST_SUCCESS;
}

//int init_intc(XScuGic* p_intc_inst, int intc_device_id, XAxiDma* p_dma_inst, int s2mm_intr_id, int mm2s_intr_id)
int initIntc(XScuGic* intc_inst, int intc_device_id)
{
	int status;
	XScuGic_Config* cfgPtr;
	// Look up hardware configuration for device
	cfgPtr = XScuGic_LookupConfig(intc_device_id);
	INIT_CONFIG_CHECK("ERROR! No hardware configuration found for Interrupt Controller with device id %d.\r\n", intc_device_id);
	// Initialize driver
	status = XScuGic_CfgInitialize(intc_inst, cfgPtr, cfgPtr->CpuBaseAddress);
	INIT_STATUS_CHECK("ERROR! Initialization of Interrupt Controller failed with %d.\r\n", status)
	status = XScuGic_SelfTest(intc_inst);
	INIT_STATUS_CHECK("ERROR! Self test failed.\r\n", status);
	// --------------------- ------------------
	// Initialize exception table and register the interrupt controller handler with exception table
	Xil_ExceptionInit();
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT, (Xil_ExceptionHandler)XScuGic_InterruptHandler, intc_inst);
	// Enable non-critical exceptions
	Xil_ExceptionEnable();
	// ----------------------------------------
	return status;
}

int intcAddInterruptHandler(Xil_InterruptHandler function, void* obj , XScuGic* intc_inst, int intc_device_id, int interrupt_id, int priority)
{
	int status;
	// Set interrupt priorities and trigger type
	// Priority is the new priority for the IRQ source. 0 is highest
	// The priority bits are Bits 7 to 3 in GIC Priority Register
	XScuGic_SetPriorityTriggerType(intc_inst, interrupt_id, priority, 0x3);
	// Connect handlers
	status = XScuGic_Connect(intc_inst, interrupt_id, (Xil_InterruptHandler)function, obj);
	INIT_STATUS_CHECK("ERROR! Failed to connect s2mm_isr to the interrupt controller.\r\n", status);
	// Enable all interrupts
	XScuGic_Enable(intc_inst, interrupt_id);
	return status;
}


const struct InitUtils initUtils = {
    .initDma = initDma,
	.initGpio = initGpio,
    .initIntc = initIntc,
    .intcAddInterruptHandler = intcAddInterruptHandler
};
