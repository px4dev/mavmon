/*
 * Copyright (c) 2012, px4dev, <px4@purgatory.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * o Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file board_fy20ap.cpp
 *
 * Board support for the FY20AP connected to a SSD1306-based I2C display.
 */

extern "C" {
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/i2c.h>
}

#include <stdio.h>
#include <errno.h>

#include "board.h"

static uint8_t com_tx_buffer[256];
static uint8_t com_rx_buffer[256];

extern "C" void usart1_isr(void);

class Board_FY20AP : public Board
{
public:
	Board_FY20AP();

	virtual void		setup();
	virtual void		com_init(unsigned speed);
	virtual void		com_fini();
	virtual void		led_set(bool state);
	virtual void		led_toggle();

protected:

	virtual void		com_tx_start(void);

private:

	friend void		usart1_isr(void);
};

static Board_FY20AP board_fy20ap;
Board *gBoard = &board_fy20ap;

Board_FY20AP::Board_FY20AP():
	Board(com_tx_buffer, sizeof(com_tx_buffer),
	      com_rx_buffer, sizeof(com_rx_buffer))
{
}

void
Board_FY20AP::setup(void)
{

	/* configure for 8MHz crystal on the FY20AP board */
	rcc_clock_setup_in_hse_8mhz_out_72mhz();

	/*
	 * Pinouts:
	 *
	 * 			OUT			IN
	 * 	NULL	NULL	RUD	ELE	AIL	SWITCH	RUD	ELE	AIL
	 * 	------------------------------------------------------------------------
	 *   A	n/c	PA10	PB6	PB7	PB9	PA0	PA1	PA3	PA4
	 *   B	Boot0	PA9	Vcc	Vcc	Vcc	Vcc	Vcc	Vcc	Vcc
	 *   C	PC8	GND	GND	GND	GND	GND	GND	GND	GND
	 *
	 * Useful pin options
	 * ------------------
	 *
	 * PA0 - TIM2_CH1, ADC12_IN0
	 * PA1 - TIM2_CH2, ADC12_IN1
	 * PA3 - TIM2_CH4, ADC12_IN3, USART2_RX
	 * PA4 - ADC12_IN4
	 *
	 * PA9  - USART1_TX, TIM1_CH2
	 * PA10 - USART1_RX, TIM1_CH3
	 *
	 * PB6 - TIM4_CH1, I2C1_SCL, USART1_TX
	 * PB7 - TIM4_CH2, I2C1_SDA, USART1_RX
	 * PB9 - TIM4_CH4
	 *
	 * PC8 - TIM3_CH3
	 *
	 * PA15 - blue LED
	 * PC12 - red LED
	 */

	/* turn on required clocks */
	rcc_peripheral_enable_clock(&RCC_APB1ENR,
				    RCC_APB1ENR_I2C1EN);
	rcc_peripheral_enable_clock(&RCC_APB2ENR,
				    RCC_APB2ENR_IOPAEN |
				    RCC_APB2ENR_IOPBEN |
				    RCC_APB2ENR_IOPCEN |
				    RCC_APB2ENR_AFIOEN |
				    RCC_APB2ENR_USART1EN);

	/* configure LED GPIOs */
	AFIO_MAPR |= AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON;
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_OPENDRAIN, GPIO15);
	gpio_set(GPIOA, GPIO15);
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_OPENDRAIN, GPIO12);
	gpio_set(GPIOC, GPIO12);

	/* configure misc GPIOs as pulled-up inputs */
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO0 | GPIO1 | GPIO3 | GPIO4);
	gpio_set(GPIOA, GPIO0 | GPIO1 | GPIO3 | GPIO4);
	gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO9);
	gpio_set(GPIOB, GPIO9);
	gpio_set_mode(GPIOC, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO8);
	gpio_set(GPIOC, GPIO8);

	/* configure USART GPIOs */
	gpio_set_mode(GPIO_BANK_USART1_TX, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);
	gpio_set_mode(GPIO_BANK_USART1_RX, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX);

	/* configure I2C GPIOs */
	gpio_set_mode(GPIO_BANK_I2C1_SCL,  GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, GPIO_I2C1_SCL);
	gpio_set_mode(GPIO_BANK_I2C1_SDA,  GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, GPIO_I2C1_SDA);

}

void
Board_FY20AP::led_toggle()
{
	gpio_toggle(GPIOA, GPIO15);
}

void
Board_FY20AP::led_set(bool state)
{
	if (state) {
		gpio_clear(GPIOA, GPIO15);

	} else {
		gpio_set(GPIOA, GPIO15);
	}
}

void
Board_FY20AP::com_init(unsigned speed)
{
	/* configure UART */
	usart_set_baudrate(USART1, speed);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
	usart_set_mode(USART1, USART_MODE_TX_RX);

	/* enable receive interrupt */
	usart_enable_rx_interrupt(USART1);

	/* and enable the UART */
	usart_enable(USART1);
}

void
Board_FY20AP::com_fini(void)
{
	usart_disable(USART1);
	usart_disable_rx_interrupt(USART1);
	usart_disable_tx_interrupt(USART1);
}

void
Board_FY20AP::com_tx_start()
{
	usart_enable_tx_interrupt(USART1);
}

OS_INTERRUPT void
usart1_isr(void)
{
	OS::scmRTOS_ISRW_TYPE ISR;

	/* receiver not empty? */
	if (usart_get_flag(USART1, USART_SR_RXNE))
		board_fy20ap.com_rx(usart_recv(USART1));

	/* transmitter ready? */
	if (usart_get_flag(USART1, USART_SR_TXE)) {

		uint8_t c;

		if (board_fy20ap.com_tx(c)) {
			usart_send(USART1, c);

		} else {
			/* clear TX empty interrupt as we have no data */
			usart_disable_tx_interrupt(USART1);
		}
	}
}

int
_write(int file, char *ptr, int len)
{
	int i;

	if (file == 1) {
		for (i = 0; i < len; i++)
			usart_send_blocking(USART1, ptr[i]);

		return i;
	}

	errno = EIO;
	return -1;
}

/****************************************************************************
 * u8g interface driver
 */

#define WIDTH 128
#define HEIGHT 32
#define PAGE_HEIGHT 8

/* init sequence adafruit 128x32 OLED (NOT TESTED) */
static const uint8_t u8g_dev_init_seq[] = {
	U8G_ESC_CS(0),
	U8G_ESC_ADR(0),
	U8G_ESC_RST(1),
	U8G_ESC_CS(1),

	0xae,			/* display off */
	0xd5, 0x80,		/* clock divide ratio (0x00=1) and oscillator frequency (0x8) */
	0xa8, 0x1f,		/* multiplexing */
	0xd3, 0x00,		/* display offset = 0 */
	0x40,			/* start line = 0 */
	0x8d, 0x14,		/* [2] charge pump setting (p62): 0x014 enable, 0x010 disable */

	0x20, 0x00,		/* memory mode */
//	0x20, 0x02,		/* 2012-05-27: page addressing mode */
	0xa1,			/* segment remap a0/a1*/
	0xc8,			/* c0: scan dir normal, c8: reverse */
	//0xda, 0x12,		/* com pin HW config, sequential com pin config (bit 4), disable left/right remap (bit 5) */
	0xda, 0x2,		/* com pin HW config, disable left/right remap (bit 5) */
//	0x81, 0xcf,		/* [2] set contrast control */
	0x81, 0x8f,		/* [2] set contrast control */
	0xd9, 0xf1,		/* [2] pre-charge period 0x022/f1*/
	0xdb, 0x40,		/* vcomh deselect level */

	0x2e,			/* 2012-05-27: Deactivate scroll */
	0xa4,			/* output ram to display */
	0xa6,			/* none inverted normal display mode */
	0xaf,			/* display on */

	U8G_ESC_CS(0),		/* disable chip */
	U8G_ESC_END		/* end of sequence */
};

static const uint8_t u8g_dev_data_start[] = {
	U8G_ESC_ADR(0),           /* instruction mode */
	U8G_ESC_CS(1),             /* enable chip */
	0x010,		/* set upper 4 bit of the col adr to 0 */
	0x000,		/* set lower 4 bit of the col adr to 4  */
	U8G_ESC_END                /* end of sequence */
};

static uint8_t current_address = 0x36;	/* I2C address, reflects the A0 flag */
static bool selected = false;		/* transaction state */

static void
com_send_start()
{
	/* send start */
	i2c_send_start(I2C1);
	while (!((I2C_SR1(I2C1) & I2C_SR1_SB)
	        & (I2C_SR2(I2C1) & (I2C_SR2_MSL | I2C_SR2_BUSY))))
	        ;

	/* send address */
	i2c_send_7bit_address(I2C1, current_address, I2C_WRITE);
	while (!(I2C_SR1(I2C1) & I2C_SR1_ADDR))
		;
	(void)I2C_SR2(I2C1);

	selected = true;
}

static void
com_send_stop()
{
	if (selected) {
		i2c_send_stop(I2C1);
		selected = false;
	}
}

static void
com_send_data(uint8_t c)
{
//	if (!selected)
		com_send_start();

	i2c_send_data(I2C1, c);
	while (!(I2C_SR1(I2C1) & I2C_SR1_BTF))
		;

	com_send_stop();
}

static uint8_t
u8g_board_com_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr)
{

	switch (msg) {
	case U8G_COM_MSG_INIT:
		//debug("u8com: init");

		/* configure I2C */
		i2c_peripheral_disable(I2C1);
		i2c_set_clock_frequency(I2C1, I2C_CR2_FREQ_36MHZ);
		i2c_set_fast_mode(I2C1);
		i2c_set_ccr(I2C1, 0x1e);
		i2c_set_trise(I2C1, 0x0b);
		i2c_set_own_7bit_slave_address(I2C1, 0x32);	/* XXX do we really want to be here? */
		i2c_peripheral_enable(I2C1);

		break;

	case U8G_COM_MSG_STOP:
		i2c_peripheral_disable(I2C1);
		break;

	case U8G_COM_MSG_ADDRESS:
		/* force a new start on address change */
//		com_send_stop();

		/* select address for command vs. data */
		current_address = arg_val ? 0x36 : 0x3c;
		break;

	case U8G_COM_MSG_CHIP_SELECT:
		if (arg_val) {
			//debug("u8com: select");
			/* do nothing here */

		} else {
			//debug("u8com: deselect");
//			com_send_stop();
		}

		break;

	case U8G_COM_MSG_RESET:
		break;

	case U8G_COM_MSG_WRITE_BYTE:
		//debug("u8com: send 0x%02x", arg_val);
		com_send_data(arg_val);
		break;

	case U8G_COM_MSG_WRITE_SEQ:
	case U8G_COM_MSG_WRITE_SEQ_P:
		//debug("u8com: seq %d", arg_val);
		{
			uint8_t *ptr = (uint8_t *)arg_ptr;

			while (arg_val-- > 0) {
				//debug("u8com:   0x%02x", *ptr);
				com_send_data(*ptr++);
			}
		}
		break;
	}

	return 1;
}

static uint8_t
u8g_board_dev_fn(u8g_t *u8g, u8g_dev_t *dev, uint8_t msg, void *arg)
{
	switch (msg) {
	case U8G_DEV_MSG_INIT:
		//debug("u8dev: init");
		u8g_InitCom(u8g, dev);
		u8g_WriteEscSeqP(u8g, dev, u8g_dev_init_seq);
		break;

	case U8G_DEV_MSG_STOP:
		break;

	case U8G_DEV_MSG_PAGE_NEXT: {
			u8g_pb_t *pb = (u8g_pb_t *)(dev->dev_mem);
			u8g_WriteEscSeqP(u8g, dev, u8g_dev_data_start);
			u8g_WriteByte(u8g, dev, 0x0b0 | pb->p.page); /* select current page (SSD1306) */
			u8g_SetAddress(u8g, dev, 1);           /* data mode */

			if (u8g_pb_WriteBuffer(pb, u8g, dev) == 0)
				return 0;

			u8g_SetChipSelect(u8g, dev, 0);
		}
		break;
	}

	return u8g_dev_pb8v1_base_fn(u8g, dev, msg, arg);
}

U8G_PB_DEV(u8g_board_dev, WIDTH, HEIGHT, PAGE_HEIGHT, u8g_board_dev_fn, u8g_board_com_fn);

/****************************************************************************
 * m2 event source
 *
 * 3 buttons, but we really want 4 ...
 */

uint8_t
m2_board_es(m2_p ep, uint8_t msg)
{
	switch (msg) {
	case M2_ES_MSG_GET_KEY:
		/* XXX need to decide how we connect buttons */
		return M2_KEY_NONE;

	case M2_ES_MSG_INIT:
		/* XXX nothing right now */
		break;
	}

	return 0;
}