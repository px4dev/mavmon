#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/spi.h>

#include "board.h"

void
board_setup(void)
{
	/* configure for 12MHz crystal on the FLD_V2 board */
	rcc_clock_setup_in_hse_12mhz_out_72mhz();

	/* turn on required clocks */
	rcc_peripheral_enable_clock(&RCC_APB2ENR, 
		RCC_APB2ENR_IOPAEN |
		RCC_APB2ENR_IOPBEN |
		RCC_APB2ENR_SPI1EN |
		RCC_APB2ENR_AFIOEN |
		RCC_APB2ENR_USART1EN);

	/* configure LED GPIO */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO11);

	/* configure buzzer GPIO and turn it off */
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO1);
	gpio_clear(GPIOB, GPIO1);

	/* configure switch GPIOs as pulled-up inputs */
	gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO10);
	gpio_set(GPIOB, GPIO10);
	gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO11);
	gpio_set(GPIOB, GPIO11);
	gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO12);
	gpio_set(GPIOB, GPIO12);

	/* configure 'spare' external pins */
	gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO13);
	gpio_set(GPIOB, GPIO13);
	gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO14);
	gpio_set(GPIOB, GPIO14);

	/* configure USART GPIOs */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX);

	/* configure SPI GPIOs */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_SPI1_SCK);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_SPI1_MOSI);

	/* configure display select/control GPIOs */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO2);	/* /RST */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO3);	/* A0 */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO4);	/* CS */

}

void
led_toggle()
{
	gpio_toggle(GPIOA, GPIO11);
}

void
serial_start(unsigned bitrate)
{
	/* configure UART */
	usart_set_baudrate(USART1, bitrate);
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
serial_stop(void)
{
	usart_disable(USART1);
	usart_disable_rx_interrupt(USART1);
	usart_disable_tx_interrupt(USART1);
}

void
usart1_isr(void)
{

	/* receiver not empty? */
	if (usart_get_flag(USART1, USART_SR_RXNE)) {
		(void)usart_recv(USART1);

		/* XXX do something with it here */
	}

	/* transmitter ready? */
	if (usart_get_flag(USART1, USART_SR_TXE)) {

		/* XXX transmit byte */

		/* clear TX empty interrupt as we have no data */
		usart_disable_tx_interrupt(USART1);
	}
}

/*
 * u8g interface drivers
 */

uint8_t
u8g_board_com_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr)
{
	switch(msg)
	{
	case U8G_COM_MSG_INIT:
		/* configure SPI */
		/*
		 * SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
		 * SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
		 * SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
		 * SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
		 * SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
		 * SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
		 * SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
		 * SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
		 * SPI_InitStructure.SPI_CRCPolynomial = 7;
		 * SPI_Init(SPI1, &SPI_InitStructure);
		 */
		spi_init_master(
			SPI1, 
			SPI_CR1_BAUDRATE_FPCLK_DIV_64,	/* XXX need to scope this - maybe able to go to 2 */
			SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE,
			SPI_CR1_CPHA_CLK_TRANSITION_1,	/* want latch-on-rising */
			SPI_CR1_DFF_8BIT,
			SPI_CR1_MSBFIRST);
		spi_enable_software_slave_management(SPI1);
		spi_enable_ss_output(SPI1);
		spi_set_nss_high(SPI1);
		spi_enable(SPI1);
		break;

	case U8G_COM_MSG_STOP:
		break;

	case U8G_COM_MSG_ADDRESS:
		if (arg_val) {
			/* display data */
			gpio_set(GPIOA, GPIO3);
		} else {
			/* display command */
			gpio_clear(GPIOA, GPIO3);
		}
		break;

	case U8G_COM_MSG_CHIP_SELECT:
		if (arg_val) {
			/* select display */
			gpio_clear(GPIOA, GPIO4);
		} else {
			/* deselect display */
			gpio_set(GPIOA, GPIO4);
		}
		break;

	case U8G_COM_MSG_RESET:
		if (arg_val) {
			/* set reset line high */
			gpio_set(GPIOA, GPIO2);
		} else {
			/* set reset line low */
			gpio_clear(GPIOA, GPIO2);
		}
		break;

	case U8G_COM_MSG_WRITE_BYTE:
		spi_send(SPI1, arg_val);
		break;

	case U8G_COM_MSG_WRITE_SEQ:
	case U8G_COM_MSG_WRITE_SEQ_P:
		{
			uint8_t *ptr = (uint8_t *)arg_ptr;

			while (arg_val-- > 0)
				spi_send(SPI1, *ptr++);
			/* XXX wait for the last byte to finish transferring... */
		}
		break;
	}
	return 1;
}

#define WIDTH 128
#define HEIGHT 64
#define PAGE_HEIGHT 8

static const uint8_t u8g_dev_init_seq[] = {
	U8G_ESC_CS(0),		/* disable chip */
	U8G_ESC_ADR(0),		/* instruction mode */
	U8G_ESC_RST(1),		/* do reset low pulse with (1*16)+2 milliseconds */
	U8G_ESC_CS(1),		/* enable chip */
	
	0xab,			/* new  */
	0xaf,			/* display on */
	0x40,			/* display start line=0 */
	0xc0,			/* Common output mode select= reverse   */
	0xa6,			/* normal display */
	0xa4,			/* Duisplay all point = off */
	0xa3,			/* LCD bias = 1/9   a2 */
	0x2f,			/* Power control = all on */
	0x25,			/* Rab Ratio     26 */
	0x81, 0x17,		/* E-Vol setting */


	U8G_ESC_DLY(100),	/* delay 100 ms */
	0x0a5,			/* display all points, ST7565 */
	U8G_ESC_DLY(100),	/* delay 100 ms */
	U8G_ESC_DLY(100),	/* delay 100 ms */
	0x0a4,			/* normal display */
	U8G_ESC_CS(0),		/* disable chip */
	U8G_ESC_END		/* end of sequence */
};

static const uint8_t u8g_dev_data_start[] = {
	U8G_ESC_ADR(0),		/* instruction mode */
	U8G_ESC_CS(1),		/* enable chip */
	0x010,			/* set upper 4 bit of the col adr to 0 */
	0x000,			/* set lower 4 bit of the col adr to 0 */      
	U8G_ESC_END		/* end of sequence */
};

uint8_t u8g_board_dev_fn(u8g_t *u8g, u8g_dev_t *dev, uint8_t msg, void *arg)
{
	switch(msg)
	{
	case U8G_DEV_MSG_INIT:
		u8g_InitCom(u8g, dev);
		u8g_WriteEscSeqP(u8g, dev, u8g_dev_init_seq);
		break;

	case U8G_DEV_MSG_STOP:
		break;

	case U8G_DEV_MSG_PAGE_NEXT:
		{
			u8g_pb_t *pb = (u8g_pb_t *)(dev->dev_mem);
			u8g_WriteEscSeqP(u8g, dev, u8g_dev_data_start);    
			u8g_WriteByte(u8g, dev, 0x0b0 | pb->p.page);	/* select current page (ST7565R) */
			u8g_SetAddress(u8g, dev, 1);			/* data mode */
			if (u8g_pb_WriteBuffer(pb, u8g, dev) == 0)
			  	return 0;
			u8g_SetChipSelect(u8g, dev, 0);
		}
		break;

	case U8G_DEV_MSG_CONTRAST:
		u8g_SetChipSelect(u8g, dev, 1);
		u8g_SetAddress(u8g, dev, 0);          /* instruction mode */
		u8g_WriteByte(u8g, dev, 0x081);
		u8g_WriteByte(u8g, dev, (*(uint8_t *)arg) >> 2);
		u8g_SetChipSelect(u8g, dev, 0);      
		return 1;
	}
	return u8g_dev_pb8v1_base_fn(u8g, dev, msg, arg);
}

U8G_PB_DEV(u8g_board_dev, WIDTH, HEIGHT, PAGE_HEIGHT, u8g_board_dev_fn, u8g_board_com_fn);

/*
 * u8g delay functions
 */
void u8g_Delay(uint16_t val)
{
	/* do not know how to delay... */
}
void u8g_MicroDelay(void)
{
}
