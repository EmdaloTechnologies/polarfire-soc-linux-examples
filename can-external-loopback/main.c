/*******************************************************************************
 * Copyright 2019-2021 Microchip FPGA Embedded Systems Solution.
 *
 * SPDX-License-Identifier: MIT
 *
 * Application code running on U54_1
 *
 * PolarFire SoC MSS CAN example demonstrating the Data transmission and
 * Reception using MSS CAN external loop back.
 * For Transmission: Get data from Hyperterminal using MSSUART1 --> Form as CAN
 * packets --> Send to CAN-0 to CAN-1.
 * For Reception: Read the received message either from CAN-0 or CAN-1 -->
 * Send to hyperterminal using MSSUART1.
 *
 * Board settings and test procedure clearly mentioned in README.md file.
 *
 */
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>

#include <fcntl.h>
#include <errno.h>
#include <poll.h>


int mhartid = 0;
int mie=0;
int read_csr(int hartid) {
	printf("reading csr: %d\n", hartid);
	return 0;
}
void set_csr(int hartid, int csr) {
	printf("setting csr for hart %d: csr: %d\n", hartid, csr);
}
#include "mpfs_hal/common/mss_peripherals.h"
uint8_t mss_config_clk_rst(mss_peripherals peripheral, uint8_t hart, PERIPH_RESET_STATE req_state) {
	printf("resetting peripheral: %d, hart: %d, state: %x\n", peripheral, hart, req_state);

	return 0;
}

#include "drivers/mss/mss_mmuart/mss_uart.h"
void MSS_UART_init ( mss_uart_instance_t* this_uart, uint32_t baud_rate, uint8_t line_config) {
	printf("initialising uart\n");
}

void MSS_UART_polled_tx_string ( mss_uart_instance_t * this_uart, const uint8_t * p_sz_string) {
	printf("%s", p_sz_string);
}

void MSS_UART_polled_tx ( mss_uart_instance_t * this_uart, const uint8_t * pbuff, uint32_t tx_size) {
	int i;

	for(i = 0; i < tx_size; i++) {
		putchar(pbuff[i]);
	}
}

size_t MSS_UART_get_rx ( mss_uart_instance_t * this_uart, uint8_t * rx_buff, size_t buff_size) {
	int i;

	for (i = 0; i < buff_size; i++) {
		rx_buff[i] = getchar();
	}

	return buff_size;
}

#include "mpfs_hal/mss_hal.h"
#include "drivers/mss/mss_can/mss_can.h"

//#include "../../platform/drivers/mss/mss_can/mss_can.c"

/*------------------------------------------------------------------------------
 * Macros.
 */
#define   ENTER_A			   0x0AU
#define   ENTER_D			   0x0DU

/*------------------------------------------------------------------------------
 * Private functions.
 */
static void display_greeting(void);
static uint8_t get_data_frm_uart(void);
static void display_hex_values(const uint8_t *, uint32_t);
static void ascii_to_hex(uint8_t *, uint32_t );
static void display_option(void);
static void check_rx_buffer(void);

/*------------------------------------------------------------------------------
 * Static Variables.
 */
static uint8_t g_uart_to_can[32] = {  0 };
static uint8_t g_temp[64] = { 0 };
static uint8_t g_can_to_uart[8] = { 0 };

/*------------------------------------------------------------------------------
 * Global Variables.
 */
mss_can_filterobject pfilter = { 0 };
mss_can_msgobject pmsg = { 0 };
mss_can_msgobject rx_buf = { 0 };
mss_can_rxmsgobject rx_msg = { 0 };

/*------------------------------------------------------------------------------
 * MSS UART instance for UART1
 */
mss_uart_instance_t *g_uart= 0;

uint64_t uart_lock = 0;

mss_can_instance_t* g_mss_can_0 = 0;
mss_can_instance_t* g_mss_can_1 = 0;

volatile uint32_t count_sw_ints_h1 = 0U;

/******************************************************************************
 *  Greeting messages displayed over the UART.
 */
const uint8_t g_greeting_msg[] =
"\n\r******************************************************************************\n\r\
******* PolarFire SoC MSS CAN Driver Example (External loop back ) ***********\n\r\
******************************************************************************\n\r\
 Example project demonstrates the use of MSS CAN Transmission and Reception \n\r\
 using external loop back. \n\r\
------------------------------------------------------------------------------\n\r\
 Read data from the UART1 and transmit as CAN message from CAN 0 to CAN 1 \n\r\
------------------------------------------------------------------------------\n\r\
 Receive the CAN Message from CAN 1 channel and send this to UART1\n\r\
******************************************************************************\n\r";

const uint8_t g_separator[] =
"\r\n----------------------------------------------------------------------\r\n";

void __enable_irq(void) {}

void clear_soft_interrupt_dummy(void) {}

void PLIC_DisableIRQ(int arse) {}

void PLIC_init(void) {}

#define SYSFS_PATH_LEN		(128)
#define ID_STR_LEN		(32)
#define UIO_DEVICE_PATH_LEN	(32)
#define NUM_UIO_DEVICES		(32)

uint32_t verbose = 0;
char uio_id_str0[] = "uiocan0";
char uio_id_str1[] = "uiocan1";
char sysfs_template[] = "/sys/class/uio/uio%d/%s";

int get_uio_device(char * id)
{
	FILE *fp;
	int i;
	int len;
	char file_id[ID_STR_LEN];
	char sysfs_path[SYSFS_PATH_LEN];

	for (i = 0; i < NUM_UIO_DEVICES; i++) {
		snprintf(sysfs_path, SYSFS_PATH_LEN, sysfs_template, i, "/name");
		fp = fopen(sysfs_path, "r");
		if (fp == NULL)
			break;

		fscanf(fp, "%32s", file_id);

		len = strlen(id);
		if (len > ID_STR_LEN-1)
			len = ID_STR_LEN-1;

		if (strncmp(file_id, id, len) == 0)
			return i;
	}

	return -1;
}


uint32_t get_memory_size(char *sysfs_path, char *devname)
{
	FILE *fp;
	uint32_t sz;

	/* 
	 * open the file the describes the memory range size.
	 * this is set up by the reg property of the node in the device tree
	 */
	fp = fopen(sysfs_path, "r");
	if (fp == NULL) {
		fprintf(stderr, "unable to determine size for %s from %s\n", devname, sysfs_path);
		exit(0);
	}

	fscanf(fp, "0x%016X", &sz);
	fclose(fp);

	return sz;
}

uint64_t get_memory_base(char *sysfs_path, char *devname)
{
	FILE *fp;
	uint64_t base;

	/* 
	 * open the file the describes the memory range base
	 * this is set up by the reg property of the node in the device tree
	 */
	fp = fopen(sysfs_path, "r");
	if (fp == NULL) {
		fprintf(stderr, "unable to determine base for %s from %s\n", devname, sysfs_path);
		exit(0);
	}

	fscanf(fp, "0x%016lX", &base);
	fclose(fp);

	return base;
}


mss_can_instance_t my_can0;
mss_can_instance_t my_can1;

/* Main function for the HART1(U54_1 processor).
 * Application code running on HART1 is placed here
 *
 * The HART1 goes into WFI. HART0 brings it out of WFI when it raises the first
 * Software interrupt to this HART
 */
int main(void)
{
	uint8_t ret_status;
	uint8_t rx_bytes = 0u;
	uint8_t no_of_msgs = 0u;
	uint8_t init_return_value = 0u;
	uint8_t rx_char, count;
	uint8_t loop_count;
	uint32_t msg_len;
	uint32_t chunk_size;
	uint32_t error_flag;
	uint32_t tx_status = 0u;
	size_t  rx_size;

	int ret = 0;
	char devname0[UIO_DEVICE_PATH_LEN];
	char devname1[UIO_DEVICE_PATH_LEN];
	char sysfs_path[SYSFS_PATH_LEN];
	uint32_t mmap_size;
	void *canptr0;
	void *canptr1;
	int index0, index1;
	int uiofd0, uiofd1;
	int64_t base0, base1;

#if (IMAGE_LOADED_BY_BOOTLOADER == 0)
	/* Clear pending software interrupt in case there was any.
	 * Enable only the software interrupt so that the E51 core can bring this
	 * core out of WFI by raising a software interrupt. */
	clear_soft_interrupt_dummy();
	set_csr(mie, MIP_MSIP);

	/* Put this hart in WFI. */
	do
	{
		__asm("wfi");
	} while (0 == (read_csr(mip) & MIP_MSIP));
#endif

	/* The hart is out of WFI, clear the SW interrupt. Here onwards Application
	 * can enable and use any interrupts as required */
	clear_soft_interrupt_dummy();

	(void)mss_config_clk_rst(MSS_PERIPH_MMUART1, (uint8_t) 1, PERIPHERAL_ON);
	(void)mss_config_clk_rst(MSS_PERIPH_CAN0, (uint8_t) 1, PERIPHERAL_ON);
	(void)mss_config_clk_rst(MSS_PERIPH_CAN1, (uint8_t) 1, PERIPHERAL_ON);

	PLIC_DisableIRQ(CAN0_PLIC);
	PLIC_DisableIRQ(CAN1_PLIC);

	PLIC_init();

	__enable_irq();


	MSS_UART_init(g_uart,
				  MSS_UART_115200_BAUD,
				  (MSS_UART_DATA_8_BITS | MSS_UART_NO_PARITY |
				  MSS_UART_ONE_STOP_BIT));

	printf("locating device for %s\n", uio_id_str0);
		index0 = get_uio_device(uio_id_str0);
	if (index0 < 0) {
		fprintf(stderr, "can't locate uio device for %s\n", uio_id_str0);
		return -1;
	}

	snprintf(devname0, UIO_DEVICE_PATH_LEN, "/dev/uio%d", index0);
	printf("located %s\n", devname0);

	printf("locating device for %s\n", uio_id_str1);
		index1 = get_uio_device(uio_id_str1);
	if (index1 < 0) {
		fprintf(stderr, "can't locate uio device for %s\n", uio_id_str1);
		return -1;
	}

	snprintf(devname1, UIO_DEVICE_PATH_LEN, "/dev/uio%d", index1);
	printf("located %s\n", devname1);

	uiofd0 = open(devname0, O_RDWR);
	if(uiofd0 < 0) {
		fprintf(stderr, "cannot open %s: %s\n", devname0, strerror(errno));
		return -1;
	} else {
		printf("opened %s (r,w)\n", devname0);
	}

	uiofd1 = open(devname1, O_RDWR);
	if(uiofd1 < 0) {
		fprintf(stderr, "cannot open %s: %s\n", devname1, strerror(errno));
		return -1;
	} else {
		printf("opened %s (r,w)\n", devname1);
	}

	/* Get base for device 0 */
	snprintf(sysfs_path, SYSFS_PATH_LEN, sysfs_template, index0, "maps/map0/addr");
	base0 = get_memory_base(sysfs_path, devname0);
	if (base0 < 0) {
		fprintf(stderr, "bad memory base for %s\n", devname0);
		return -1;
	}

	/* Get base for device 1 */
	snprintf(sysfs_path, SYSFS_PATH_LEN, sysfs_template, index1, "maps/map0/addr");
	base1 = get_memory_base(sysfs_path, devname0);
	if (base1 < 0) {
		fprintf(stderr, "bad memory base for %s\n", devname1);
		return -1;
	}

	/* Get size */
	snprintf(sysfs_path, SYSFS_PATH_LEN, sysfs_template, index0, "maps/map0/size");
	mmap_size = get_memory_size(sysfs_path, devname0);
	if (mmap_size == 0) {
		fprintf(stderr, "bad memory size for %s\n", devname0);
		return -1;
	}

	canptr0 = mmap(NULL, mmap_size, PROT_READ|PROT_WRITE, MAP_SHARED, uiofd0, 0);
	if (canptr0 == MAP_FAILED) {
		fprintf(stderr, "cannot mmap %s: %s\n", devname0, strerror(errno));
		return -1;
	} else {
		printf("mapped 0x%x bytes for %s (phys: 0x%lx)\n", mmap_size, devname0, base0);
	}

	snprintf(sysfs_path, SYSFS_PATH_LEN, sysfs_template, index1, "maps/map0/size");
	mmap_size = get_memory_size(sysfs_path, devname1);
	if (mmap_size == 0) {
		fprintf(stderr, "bad memory size for %s\n", devname1);
		return -1;
	}

	canptr1 = mmap(NULL, mmap_size, PROT_READ|PROT_WRITE, MAP_SHARED, uiofd1, 0);
	if (canptr1 == MAP_FAILED) {
		fprintf(stderr, "cannot mmap %s: %s\n", devname1, strerror(errno));
		return -1;
	} else {
		printf("mapped 0x%x bytes for %s (phyx: 0x%lx)\n", mmap_size, devname1, base0);
	}

	g_mss_can_0 = &my_can0;
	g_mss_can_1 = &my_can1;

	g_mss_can_0->hw_reg = (CAN_DEVICE *)canptr0;
	g_mss_can_0->irqn = 56;
	g_mss_can_0->int_type = 1;
	g_mss_can_0->basic_can_rx_mb = 32;
	g_mss_can_0->basic_can_tx_mb = 32;

	g_mss_can_1->hw_reg = (CAN_DEVICE *)canptr1;
	g_mss_can_0->irqn = 57;
	g_mss_can_0->int_type = 1;
	g_mss_can_0->basic_can_rx_mb = 32;
	g_mss_can_0->basic_can_tx_mb = 32;

	/*--------------------------------------------------------------------------
	 * Performs CAN Initialization and Message Buffer Configuration
	 */
	/* ----------------------- CAN - 0 Initialization   ----------------- */
	init_return_value = MSS_CAN_init(g_mss_can_0, CAN_SPEED_8M_250K, (pmss_can_config_reg)0, 6u, 6u);
	if ((init_return_value != CAN_OK) && (init_return_value != CAN_BASIC_CAN_MAILBOX)) {
		fprintf(stderr, "can 0 failed to init ok\n");
		exit(0);
	}

	MSS_CAN_set_mode(g_mss_can_0, CANOP_MODE_NORMAL);

	MSS_CAN_start(g_mss_can_0);


	/* ----------------------- CAN - 1 Initialization   ----------------- */
	init_return_value = MSS_CAN_init(g_mss_can_1, CAN_SPEED_8M_250K, (pmss_can_config_reg)0, 6u, 6u);
	if ((init_return_value != CAN_OK) && (init_return_value != CAN_BASIC_CAN_MAILBOX)) {
		fprintf(stderr, "can 1 failed to init ok\n");
		exit(0);
	}

	MSS_CAN_set_mode(g_mss_can_1, CANOP_MODE_NORMAL);


	MSS_CAN_start(g_mss_can_1);

	/* Display greeting message */
	display_greeting();

	/* Clear receive buffer */
	for (count = 0u; count < 8u; count++)
	{
		rx_buf.DATA[count] = 0u;
	}

	/* Configure for transmit */
	pmsg.ID = 0x78u;
	pmsg.DATALOW = 0xAAAAAAAAu;
	pmsg.DATAHIGH = 0x55555555u;
#ifdef CAN_TX_EXTENDED_ID
	pmsg.L =((1<<20) | 0x00080000u); /* Extended ID, 8 bytes of data */
#else
	pmsg.L = 0x00080000u;		  /* Standard ID, 8 bytes of data */
#endif

	/* Configure for receive */
	/* Initialize the rx mailbox */
	rx_msg.ID = 0x80u;
	rx_msg.DATAHIGH = 0u;
	rx_msg.DATALOW = 0u;
	rx_msg.AMR.L = 0xFFFFFFFFu;
	rx_msg.ACR.L = 0x00000000u;
	rx_msg.AMR_D = 0xFFFFFFFFu;
	rx_msg.ACR_D = 0x00000000u;
	rx_msg.RXB.DLC = 8u;
	rx_msg.RXB.IDE = 0u;

	/* Configure receive buffer For CAN 0 */
	ret_status = MSS_CAN_config_buffer_n(g_mss_can_0, 0, &rx_msg);

	if (CAN_OK != ret_status)
	{
		MSS_UART_polled_tx_string(g_uart,
			   (const uint8_t *)"\n\r CAN 0 Message Buffer configuration Error");
	}


	/* Configure receive buffer For CAN 1 */
	rx_msg.ID = 0x78u;
	ret_status = MSS_CAN_config_buffer_n(g_mss_can_1, 1, &rx_msg);
	if (CAN_OK != ret_status)
	{
		MSS_UART_polled_tx_string(g_uart,
			   (const uint8_t *)"\n\r CAN 1 Message Buffer configuration Error");
	}

	while (1)
	{
		/*----------------------------------------------------------------------
		 * Read the Data from UART and Transmit using CAN
		 */
		rx_bytes = get_data_frm_uart();

		printf("rx_bytes: %d\n", rx_bytes);

		/* Convert ASCII values to Hex */
		ascii_to_hex(g_temp, rx_bytes);

		for (loop_count = 0u; loop_count < rx_bytes / 2u; loop_count++)
		{
			g_uart_to_can[loop_count] = g_temp[loop_count * 2u];
			g_uart_to_can[loop_count] = g_uart_to_can[loop_count] << 4u;
			g_uart_to_can[loop_count] |= g_temp[(loop_count * 2u) + 1u];
		}
		MSS_UART_polled_tx_string(g_uart,
					   (const uint8_t *)"\n\r Data transmitted as CAN Message ");
		display_hex_values(g_uart_to_can, loop_count);

		/*------------------------------------------------------------------
		  Identify the number of messages to transmit based on rx_bytes
		 */
		no_of_msgs = rx_bytes / 16u;
		if ((rx_bytes % 16u) != 0)
		{
			no_of_msgs = no_of_msgs + 1u;
		}

		if (0u == loop_count) /* Allow sending an empty packet */
		{
			no_of_msgs = 1u;
		}

		count = 0u;

		msg_len = loop_count;
		error_flag = 0u;
		while ((no_of_msgs != 0u) && (0u == error_flag))
		{
			 /* Pack up to 8 bytes into this packet in 2 x 32bit chunks */
			if (msg_len >= 8u)
			{
				chunk_size = 8u;
			}
			else
			{
				chunk_size = msg_len;
			}

			for (loop_count = 0u; loop_count < chunk_size; loop_count++)
			{
				if (loop_count < 4u)
				{
					pmsg.DATA[3u - loop_count] =  \
									   g_uart_to_can[loop_count +(count * 8u)];
				}
				else
				{
					pmsg.DATA[11u - loop_count] = \
									   g_uart_to_can[loop_count + (count * 8u)];
				}
			}

			pmsg.DLC = chunk_size;

			ret_status = MSS_CAN_send_message_n(g_mss_can_0, 0u, &pmsg);

			if (CAN_VALID_MSG != ret_status)
			{
				error_flag = 1; /* Didn't succeed in sending packet... */
			}
			else
			{
				/* Wait around for this packet to send before going any
				 * further */
				tx_status = MSS_CAN_get_tx_buffer_status(g_mss_can_0);
				while (1u == (tx_status & 1))
				{
					tx_status = MSS_CAN_get_tx_buffer_status(g_mss_can_0);
				}

			no_of_msgs--;
			msg_len -= chunk_size;
			count++;
			}
		}

		if (0u == count) /* Nothing sent */
		{
			MSS_UART_polled_tx_string(g_uart,
					(const uint8_t *)"\n\r Unable to send data via CAN Bus\n");
		}
		else
		{
			if (0u == error_flag) /* Everything sent */
			{
				MSS_UART_polled_tx_string(g_uart,
					(const uint8_t *)"\n\r Observe the data received on CAN1\n");
				MSS_UART_polled_tx_string(g_uart,
					 (const uint8_t *)"\n\r It should be same as the data "
							 "transmitted from Hyperterminal\n");
			}
			else /* Some error occurred */
			{
				MSS_UART_polled_tx_string(g_uart,
					(const uint8_t *)"\n\r Observe the data received on CAN1\n");
				MSS_UART_polled_tx_string(g_uart,
					(const uint8_t *)"\n\r Some transmission error(s) were "
								"detected.\n");
			}
		}

		MSS_UART_polled_tx_string (g_uart, g_separator);
		MSS_UART_polled_tx_string(g_uart, (const uint8_t*)"\n\r Press any key "
				"to continue...");

		do {
			rx_size = MSS_UART_get_rx(g_uart, &rx_char, sizeof(rx_char));
		} while (rx_size == 0u);

		 MSS_UART_polled_tx_string(g_uart, (const uint8_t*)"\n\r");

	   /*----------------------------------------------------------------------
		*  Display options
		*/
		display_option();
	}

	return ret;
}

static void check_rx_buffer(void)
{
	uint8_t loop_count;

	/*----------------------------------------------------------------------
	  Read the Data from CAN channel and Transmit Through UART
	 */
	if (CAN_VALID_MSG == MSS_CAN_get_message_n(g_mss_can_0, 0u, &rx_buf))
	{
		for (loop_count = 0u; loop_count < rx_buf.DLC; loop_count++)
		{
			if (loop_count < 4u)
			{
				g_can_to_uart[loop_count] = rx_buf.DATA[3u - loop_count];
			}
			else
			{
				g_can_to_uart[loop_count] = rx_buf.DATA[11u - loop_count];
			}
		}

		MSS_UART_polled_tx_string (g_uart, g_separator);
		MSS_UART_polled_tx_string(g_uart, (const uint8_t *)"\n\r Data received "
				"as CAN0 message is ");
		MSS_UART_polled_tx_string(g_uart, (const uint8_t*)"\n\r ");

		/* Send to UART */
		display_hex_values(g_can_to_uart, rx_buf.DLC);
		MSS_UART_polled_tx_string(g_uart, (const uint8_t *)"\n\r Observe the "
				"message sent from the CAN Analyzer ");
		MSS_UART_polled_tx_string(g_uart, (const uint8_t *)"\n\r It should be "
				"same as message received on Hyperterminal");
		MSS_UART_polled_tx_string (g_uart, g_separator);
	}
}

static void can1_check_rx_buffer(void)
{
	uint8_t loop_count;

	/*----------------------------------------------------------------------
	  Read the Data from CAN channel and Transmit Through UART
	 */
	if (CAN_VALID_MSG == MSS_CAN_get_message_n(g_mss_can_1, 1u, &rx_buf))
	{
		for (loop_count = 0u; loop_count < rx_buf.DLC; loop_count++)
		{
			if (loop_count < 4u)
			{
				g_can_to_uart[loop_count] = rx_buf.DATA[3u - loop_count];
			}
			else
			{
				g_can_to_uart[loop_count] = rx_buf.DATA[11u - loop_count];
			}
		}

		MSS_UART_polled_tx_string (g_uart, g_separator);
		MSS_UART_polled_tx_string(g_uart, (const uint8_t *)"\n\r Data Received "
				"as CAN1 Message is ");
		MSS_UART_polled_tx_string(g_uart, (const uint8_t*)"\n\r ");

		/* Send to UART */
		display_hex_values(g_can_to_uart, rx_buf.DLC);
		MSS_UART_polled_tx_string(g_uart, (const uint8_t *)"\n\r Observe the "
				"message sent from CAN0 ");
		MSS_UART_polled_tx_string(g_uart, (const uint8_t *)"\n\r It should be "
				"same as message received on Hyperterminal");
		MSS_UART_polled_tx_string (g_uart, g_separator);
	}
}

/*------------------------------------------------------------------------------
  Receive the data from UART
 */
static uint8_t get_data_frm_uart(void)
{
	uint8_t complete = 0;
	uint8_t rx_buff[1];
	uint8_t count = 0;
	uint8_t rx_size = 0;

	for (count = 0u; count < 32u; count++)
	{
		g_uart_to_can[count] = 0u;
	}

	MSS_UART_polled_tx_string(g_uart, (const uint8_t *)"\r\n Enter the data to "
			"transmit through the CAN Channel:\r\n ");

	count = 0u;
	while (!complete)
	{
		rx_size = MSS_UART_get_rx(g_uart, rx_buff, sizeof(rx_buff));
		if(rx_size > 0u)
		{
			MSS_UART_polled_tx(g_uart, rx_buff, sizeof(rx_buff));

			if (ENTER_A == rx_buff[0])
			{
				complete = 1u;
			}
			else
			{
				if (count % 2u == 0u)
				{
					g_temp[count] = rx_buff[0];
				}
				else
				{
					g_temp[count] = rx_buff[0];
				}

				count++;
			}

			if (64u == count)
			{
				complete = 1u;
			}
		}
	}

	return(count);
}

/*------------------------------------------------------------------------------
  Display greeting message when application is started.
 */
static void display_greeting(void)
{
	MSS_UART_polled_tx_string (g_uart, g_greeting_msg);
}

/*------------------------------------------------------------------------------
  Display content of buffer passed as parameter as hex values
 */
static void display_hex_values
(
	const uint8_t * in_buffer,
	uint32_t byte_length
)
{
	uint8_t display_buffer[128];
	uint32_t inc;

	if (0u == byte_length)
	{
		MSS_UART_polled_tx_string(g_uart, (const uint8_t*)" <No data present>\n\r");
	}
	else
	{
		if (byte_length > 16u)
		{
			MSS_UART_polled_tx_string(g_uart, (const uint8_t*)"\n\r ");
		}

		for (inc = 0u; inc < byte_length; ++inc)
		{
			if ((inc > 1u) && (0u == (inc % 16u)))
			{
				MSS_UART_polled_tx_string(g_uart, (const uint8_t*)"\n\r");
			}
			snprintf((char *)display_buffer, sizeof(display_buffer), "%02x ",
					in_buffer[inc]);
			MSS_UART_polled_tx_string(g_uart, display_buffer);
		}
	}
}

/*------------------------------------------------------------------------------
  Converts ASCII values to HEX values
 */
static void ascii_to_hex
(
	uint8_t * in_buffer,
	uint32_t byte_length
)
{
	uint32_t inc;

	for (inc = 0u; inc < byte_length; inc++)
	{
		if ((in_buffer[inc] <= 0x39u) && (in_buffer[inc] >= 0x30u))
		{
			in_buffer[inc] = in_buffer[inc] - 0x30u;
		}
		else if ((in_buffer[inc] <= 0x5Au) && (in_buffer[inc] >= 0x41u))
		{
			in_buffer[inc] = 0x0Au + in_buffer[inc] - 0x41u;
		}
		else if ((in_buffer[inc] <= 0x7Au) && (in_buffer[inc] >= 0x61u))
		{
			in_buffer[inc] = 0x0au + (in_buffer[inc] - 0x61u);
		}
		else
		{
		}
	}
}

/*------------------------------------------------------------------------------
  Display the Option to continue or exit.
 */
static void display_option(void)
{
	uint8_t rx_size=0;
	uint8_t rx_buff[1];

	/*----------------------------------------------------------------------
	  Read the Data from CAN channel and Transmit Through UART1
	*/
	check_rx_buffer();
	can1_check_rx_buffer();

	MSS_UART_polled_tx_string (g_uart, g_separator);
	MSS_UART_polled_tx_string(g_uart, (const uint8_t*)"\n\r Select the Option "
			"to proceed further \n\r");
	MSS_UART_polled_tx_string(g_uart, (const uint8_t*)" Press Key '7' to send "
			" data.\n\r");
	MSS_UART_polled_tx_string(g_uart, (const uint8_t*)" Press Key 'q' to exit "
			" \n\r");
	MSS_UART_polled_tx_string (g_uart, g_separator);
	do
	{
		/* Start command line interface if any key is pressed. */
		rx_size = MSS_UART_get_rx(g_uart, rx_buff, sizeof(rx_buff));
		if (rx_size > 0u)
		{
			switch(rx_buff[0])
			{
				case '7':
				break;

				case 'q':
				exit(0);
				break;

				default:
				break;
			}
		}

	} while (rx_buff[0]!= '7');

	/* buffered input - take the 'cr' from getchar() */
	rx_size = MSS_UART_get_rx(g_uart, rx_buff, sizeof(rx_buff));
	printf("'%c'", rx_buff[0]);
}
