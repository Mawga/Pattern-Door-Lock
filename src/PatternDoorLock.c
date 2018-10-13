/*
 * @brief I2C example
 * This example show how to use the I2C interface
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2012
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

#include <stdlib.h>
#include <string.h>
#include <board.h>


/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/
#define DEFAULT_I2C          I2C0

#define I2C_EEPROM_BUS       DEFAULT_I2C
#define I2C_IOX_BUS          DEFAULT_I2C

#define SPEED_100KHZ         100000
#define SPEED_400KHZ         400000

#define TIMER0_IRQ_HANDLER				TIMER0_IRQHandler  // TIMER0 interrupt IRQ function name
#define TIMER0_INTERRUPT_NVIC_NAME		TIMER0_IRQn        // TIMER0 interrupt NVIC interrupt name

#define TIMER1_IRQ_HANDLER				TIMER1_IRQHandler  // TIMER0 interrupt IRQ function name
#define TIMER1_INTERRUPT_NVIC_NAME		TIMER1_IRQn        // TIMER0 interrupt NVIC interrupt name


static int mode_poll;   /* Poll/Interrupt mode flag */
static I2C_ID_T i2cDev = DEFAULT_I2C; /* Currently active I2C device */

/* EEPROM SLAVE data */
#define I2C_SLAVE_EEPROM_SIZE       64
#define I2C_SLAVE_EEPROM_ADDR       0x5A
#define I2C_SLAVE_TEMP_ADDR          0x38

/* Xfer structure for slave operations */
static I2C_XFER_T temp_xfer;
static I2C_XFER_T iox_xfer;

static uint8_t i2Cbuffer[2][256];
uint8_t rows[4] = {0b00000000, 0b00000000, 0b00000000, 0b00000000};
uint8_t passRows[4] = {0b00000000, 0b00000000, 0b00000000, 0b00000000};
int rowSelect = 0;
int joyDebounce = 1;
int hasPassword = 0;
int blinkCounter = 0;
int matchCounter = 0;
int blink = 0;


/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* State machine handler for I2C0 and I2C1 */
static void i2c_state_handling(I2C_ID_T id)
{
	if (Chip_I2C_IsMasterActive(id)) {
		Chip_I2C_MasterStateHandler(id);
	} else {
		Chip_I2C_SlaveStateHandler(id);
	}
}

/* Set I2C mode to polling/interrupt */
static void i2c_set_mode(I2C_ID_T id, int polling)
{
	if(!polling) {
		mode_poll &= ~(1 << id);
		Chip_I2C_SetMasterEventHandler(id, Chip_I2C_EventHandler);
		NVIC_EnableIRQ(id == I2C0 ? I2C0_IRQn : I2C1_IRQn);
	} else {
		mode_poll |= 1 << id;
		NVIC_DisableIRQ(id == I2C0 ? I2C0_IRQn : I2C1_IRQn);
		Chip_I2C_SetMasterEventHandler(id, Chip_I2C_EventHandlerPolling);
	}
}

void TIMER0_IRQHandler(void)
{
	if (Chip_TIMER_MatchPending(LPC_TIMER0, 0))
	{
		Chip_TIMER_ClearMatch(LPC_TIMER0,0);  // Clear TIMER0 interrupt
		joyDebounce = 1;
		Chip_TIMER_Reset(LPC_TIMER0);
		Chip_TIMER_Disable(LPC_TIMER0);
	}
}

void TIMER1_IRQHandler(void)
{
	if (Chip_TIMER_MatchPending(LPC_TIMER1, 1))
	{
		Chip_TIMER_ClearMatch(LPC_TIMER1,1);
		++blinkCounter;
	}
	if (Chip_TIMER_MatchPending(LPC_TIMER1, 2))
	{
		Chip_TIMER_ClearMatch(LPC_TIMER1,2);
		++blinkCounter;
		Chip_TIMER_Reset(LPC_TIMER1);
	}
}

/* Initialize the I2C bus */
static void i2c_app_init(I2C_ID_T id, int speed)
{
	Board_I2C_Init(id);

	/* Initialize I2C */
	Chip_I2C_Init(id);
	Chip_I2C_SetClockRate(id, speed);

	/* Set default mode to interrupt */
	i2c_set_mode(id, 0);
}

static void i2c_write_setup(I2C_XFER_T *xfer, uint8_t addr, int numBytes)
{
	xfer->slaveAddr = addr;
	xfer->rxBuff = 0;
	xfer->txBuff = 0;
	xfer->rxSz = 0;
	xfer->txSz = numBytes;
	xfer->txBuff = i2Cbuffer[0];

}


/*****************************************************************************
 * Public functions
 ****************************************************************************/
/**
 * @brief	SysTick Interrupt Handler
 * @return	Nothing
 * @note	Systick interrupt handler updates the button status
 */

void confirmButton(void)
{
	rowSelect = 0;
	if (hasPassword == 0)
	{
		hasPassword = 1;
		passRows[0] = rows[0];
		passRows[1] = rows[1];
		passRows[2] = rows[2];
		passRows[3] = rows[3];
		matchCounter = 1;
		DEBUGOUT("Password Set\r\n");
	} else {
		if(passRows[0] == rows[0] && passRows[1] == rows[1] && passRows[2] == rows[2] &&
				passRows[3] == rows[3]) {
			Chip_GPIO_SetPinOutHigh(LPC_GPIO, 1, 13);
			DEBUGOUT("Correct Password\r\n");
			matchCounter = 5;
		} else {
			DEBUGOUT("Wrong Password\r\n");
			matchCounter = 3;
		}
	}
	rows[0] = rows[1] = rows[2] = rows[3] = 0b0;
	blink = 1;
}
void drawRow(int status)
{
	switch (status)
	{
	case 0b0001: //1 right
		if (rows[rowSelect] != 0b11111111)
			rows[rowSelect] = rightShift(rows[rowSelect]);
		break;
	case 0b0010: //2 left
		if (rows[rowSelect] != 0b0)
			rows[rowSelect] = leftShift(rows[rowSelect]);
		break;
	case 0b0100: //4 up
		if (rowSelect > 0)
			--rowSelect;
		break;
	case 0b1000: //8 down
		if (rowSelect < 3)
			++rowSelect;
		break;
	}
	if (status == 0b0001 || status == 0b0010) {
		if (rowSelect == 0)
			i2Cbuffer[0][1] = i2Cbuffer[0][3] = rows[rowSelect];
		else if (rowSelect == 1)
			i2Cbuffer[0][5] = i2Cbuffer[0][7] = rows[rowSelect];
		else if (rowSelect == 2)
			i2Cbuffer[0][9] = i2Cbuffer[0][11] = rows[rowSelect];
		else if (rowSelect == 3)
			i2Cbuffer[0][13] = i2Cbuffer[0][15] = rows[rowSelect];
		Chip_I2C_MasterSend(i2cDev, temp_xfer.slaveAddr, temp_xfer.txBuff, 16);
	}
}

int leftShift(int number)
{
	if (number == 0b10000000)
		number = 0b00000000;
	else if (number == 0b10000001)
		number = 0b10000000;
	else {
		number &= 0b01111111;
		number = number >> 1;
		number |= 0b10000001;
	}
	return number;
}
int rightShift(int number)
{
	if (number == 0b0)
		number = 0b10000000;
	else {
		number = number << 1;
		number |= 0b10000001;
	}
	return number;
}

void SysTick_Handler(void)
{
}

/**
 * @brief	I2C Interrupt Handler
 * @return	None
 */
void I2C1_IRQHandler(void)
{
	i2c_state_handling(I2C1);
}

/**
 * @brief	I2C0 Interrupt handler
 * @return	None
 */
void I2C0_IRQHandler(void)
{
	i2c_state_handling(I2C0);
}

/**
 * @brief	Main program body
 * @return	int
 */
int main(void)
{
	int PrescaleValue = 120000;

	Chip_TIMER_Init(LPC_TIMER0);					   // Initialize TIMER0
	Chip_TIMER_PrescaleSet(LPC_TIMER0,PrescaleValue);  // Set prescale value
	Chip_TIMER_SetMatch(LPC_TIMER0,0,100);		    // Set match value
	Chip_TIMER_MatchEnableInt(LPC_TIMER0, 0);		   // Configure to trigger interrupt on match

	Chip_TIMER_Init(LPC_TIMER1);					   // Initialize TIMER0
	Chip_TIMER_PrescaleSet(LPC_TIMER1,PrescaleValue);  // Set prescale value
	Chip_TIMER_SetMatch(LPC_TIMER1,1,250);		    // Set match value
	Chip_TIMER_MatchEnableInt(LPC_TIMER1, 1);		   // Configure to trigger interrupt on match

	Chip_TIMER_SetMatch(LPC_TIMER1,2,500);		    // Set match value
	Chip_TIMER_MatchEnableInt(LPC_TIMER1, 2);		   // Configure to trigger interrupt on match

	NVIC_ClearPendingIRQ(TIMER1_INTERRUPT_NVIC_NAME);
	NVIC_EnableIRQ(TIMER1_INTERRUPT_NVIC_NAME);

	NVIC_ClearPendingIRQ(TIMER0_INTERRUPT_NVIC_NAME);
	NVIC_EnableIRQ(TIMER0_INTERRUPT_NVIC_NAME);

	int tmp;
	int xflag = 0;

	int count;

	Board_Init();
	SystemCoreClockUpdate();
	Board_Joystick_Init();
	i2c_app_init(I2C0, SPEED_100KHZ);
	i2c_set_mode(I2C0, 0);
	i2cDev = I2C0;
	int i;

	//for (i = 0; i < 10; ++i) {
	//Chip_TIMER_Enable(LPC_TIMER0); //Enable timer to count process time of I2C


	i2c_write_setup(&temp_xfer, ((I2C_SLAVE_TEMP_ADDR << 1)), 1);
	i2Cbuffer[0][0] = 0x21;
	Chip_I2C_MasterSend(i2cDev, temp_xfer.slaveAddr, temp_xfer.txBuff, 1);
	uint8_t b = 0;
	i2Cbuffer[0][0] = (0x80 | 0x01 | (b << 1));
	Chip_I2C_MasterSend(i2cDev, temp_xfer.slaveAddr, temp_xfer.txBuff, 1);
	b = 15;
	i2Cbuffer[0][0] = (0xE0 | b);
	Chip_I2C_MasterSend(i2cDev, temp_xfer.slaveAddr, temp_xfer.txBuff, 1);


	i2c_write_setup(&temp_xfer, ((I2C_SLAVE_TEMP_ADDR << 1)), 16);
	i2Cbuffer[0][0] = 0b00000000;
	i2Cbuffer[0][1] = 0b00000000;//Row 1
	i2Cbuffer[0][2] = 0b00000000;
	i2Cbuffer[0][3] = 0b00000000;//Row 2
	i2Cbuffer[0][4] = 0b00000000;
	i2Cbuffer[0][5] = 0b00000000;//Row 3
	i2Cbuffer[0][6] = 0b00000000;
	i2Cbuffer[0][7] = 0b00000000;//Row 4
	i2Cbuffer[0][8] = 0b00000000;
	i2Cbuffer[0][9] = 0b00000000;//Row 5
	i2Cbuffer[0][10] = 0b00000000;
	i2Cbuffer[0][11] = 0b00000000;//Row 6
	i2Cbuffer[0][12] = 0b00000000;
	i2Cbuffer[0][13] = 0b00000000;//Row 7
	i2Cbuffer[0][14] = 0b00000000;
	i2Cbuffer[0][15] = 0b00000000;//Row 8
	Chip_I2C_MasterSend(i2cDev, temp_xfer.slaveAddr, temp_xfer.txBuff, 16);

	Chip_GPIO_SetPinDIROutput(LPC_GPIO, 1, 13);


	//printf("Read %d bytes of data from slave 0x%02X.\r\n", tmp, temp_xfer.slaveAddr >> 1);
	//char textBuff[5] = "   \n";
	//memcpy(textBuff, i2Cbuffer, 2);
	//printf("%d\r\n", i2Cbuffer[1][0]);

	//Chip_TIMER_Disable(LPC_TIMER0);
	//count = Chip_TIMER_ReadCount(LPC_TIMER0);
	//Chip_TIMER_Reset(LPC_TIMER0);

	//printf("%d\r\n", count);
	//}
	while(1)
	{
		int status = Joystick_GetStatus();
		if (blink) {
			if (blinkCounter%2 == 0) {
				i2Cbuffer[0][1] = i2Cbuffer[0][3] =
					i2Cbuffer[0][5] = i2Cbuffer[0][7] =
							i2Cbuffer[0][9] = i2Cbuffer[0][11] =
									i2Cbuffer[0][13] = i2Cbuffer[0][15] = 0b11111111;
				Chip_I2C_MasterSend(i2cDev, temp_xfer.slaveAddr, temp_xfer.txBuff, 16);
			}
			else {
				i2Cbuffer[0][1] = i2Cbuffer[0][3] =
					i2Cbuffer[0][5] = i2Cbuffer[0][7] =
						i2Cbuffer[0][9] = i2Cbuffer[0][11] =
							i2Cbuffer[0][13] = i2Cbuffer[0][15] = 0b0;
				Chip_I2C_MasterSend(i2cDev, temp_xfer.slaveAddr, temp_xfer.txBuff, 16);
				if (blinkCounter >= matchCounter)
				{
					blink = 0;
					blinkCounter = 0;
					Chip_TIMER_Disable(LPC_TIMER1);
					Chip_TIMER_Reset(LPC_TIMER1);
				}
			}
		}
		else if (status == 0b10000) {
			confirmButton();
			Chip_TIMER_Enable(LPC_TIMER1);
		}
		else if (status && joyDebounce) {
			joyDebounce = 0;
			drawRow(status);
			Chip_TIMER_Enable(LPC_TIMER0);
		}
	}
	Chip_I2C_DeInit(I2C0);


	return 0;
}
