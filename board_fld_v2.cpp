extern "C" {
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/spi.h>
}

#include <stdio.h>
#include <errno.h>

#include "board.h"

#define TX_BUFFER_SIZE	256
#define RX_BUFFER_SIZE	256

static uint8_t tx_buffer_bytes[256];
static RingBuffer tx_buffer(tx_buffer_bytes, sizeof(tx_buffer_bytes));
static uint8_t rx_buffer_bytes[256];
static RingBuffer rx_buffer(rx_buffer_bytes, sizeof(rx_buffer_bytes));

class Board_FLD_V2 : public Board
{
public:
	Board_FLD_V2();

	virtual void		setup();
	virtual void		com_init(unsigned speed);
	virtual void		com_fini();
	virtual void		led_set(bool state);
	virtual void		led_toggle();

	void			com_rx(uint8_t c);
	bool			com_tx(uint8_t &c);

protected:
	virtual void		com_tx_start(void);

private:
	uint8_t			_tx_buffer_bytes[TX_BUFFER_SIZE];
	RingBuffer		_tx_buffer;
	uint8_t			_rx_buffer_bytes[RX_BUFFER_SIZE];
	RingBuffer		_rx_buffer;
};

static Board_FLD_V2 board_fld_v2;
Board *gBoard = &board_fld_v2;

Board_FLD_V2::Board_FLD_V2():
	Board(&_tx_buffer, &_rx_buffer),
	_tx_buffer(_tx_buffer_bytes, TX_BUFFER_SIZE),
	_rx_buffer(_rx_buffer_bytes, RX_BUFFER_SIZE)
{
}

void
Board_FLD_V2::setup(void)
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
	gpio_set_mode(GPIO_BANK_SPI1_SCK,  GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_SPI1_SCK);
	gpio_set_mode(GPIO_BANK_SPI1_MOSI, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_SPI1_MOSI);

	/* configure display select/control GPIOs */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO2);	/* /RST */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO3);	/* A0 */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO4);	/* CS */

}

void
Board_FLD_V2::led_toggle()
{
	gpio_toggle(GPIOA, GPIO11);
}

void
Board_FLD_V2::led_set(bool state)
{
	if (state) {
		gpio_set(GPIOA, GPIO11);

	} else {
		gpio_clear(GPIOA, GPIO11);
	}
}

void
Board_FLD_V2::com_init(unsigned speed)
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
Board_FLD_V2::com_fini(void)
{
	usart_disable(USART1);
	usart_disable_rx_interrupt(USART1);
	usart_disable_tx_interrupt(USART1);
}

void
Board_FLD_V2::com_tx_start()
{
	usart_enable_tx_interrupt(USART1);
}

void
Board_FLD_V2::com_rx(uint8_t c)
{
	_rx_buffer.insert(c);

	/* wake anyone that might be waiting */
	_rx_data_avail.signal_isr();
}

bool
Board_FLD_V2::com_tx(uint8_t &c)
{
	unsigned avail = _tx_buffer.contains();

	/* if there is no more data to send, bail now */
	if (avail == 0)
		return false;

	/*
	 * Mitigate writer wakeup costs by only signalling that there is
	 * more TX space when at least 8 bytes are free.
	 */
	if ((_tx_buffer.capacity() - avail) > 8)
		_tx_space_avail.signal_isr();

	/*
	 * Get the byte we're going to send.
	 */
	c = _tx_buffer.remove();
	return true;
}

extern "C" void
usart1_isr(void)
{
	/* receiver not empty? */
	if (usart_get_flag(USART1, USART_SR_RXNE))
		board_fld_v2.com_rx(usart_recv(USART1));

	/* transmitter ready? */
	if (usart_get_flag(USART1, USART_SR_TXE)) {

		uint8_t c;

		if (board_fld_v2.com_tx(c)) {
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
 * u8g interface drivers
 */

#define WIDTH 128
#define HEIGHT 64
#define PAGE_HEIGHT 8

static const uint8_t u8g_dev_init_seq[] = {
	U8G_ESC_CS(0),		/* de-select display */
	U8G_ESC_ADR(0),		/* instruction mode */
	U8G_ESC_RST(1),		/* do reset low pulse with (1*16)+2 milliseconds */
	U8G_ESC_DLY(1),		/* delay 1 ms after releasing reset */
	U8G_ESC_CS(1),		/* select display */

	0xaf,			/* display on */
	0x40,			/* display start line = 0 */
	0xc8,			/* COM scan direction (reverse) */
	0xa6,			/* normal display */
	0xa0,			/* scan in normal direction */
	0xa4,			/* clear display */
	0xa2,			/* LCD bias = 1/9 */
	0x2f,			/* Power control = all on */
	0x23,			/* Rab Ratio  */
	0x81, 0x25,		/* E-Vol setting */

	U8G_ESC_DLY(100),	/* delay 100 ms */
	0xa5,			/* all-pixels-on */
	U8G_ESC_DLY(100),	/* delay 100 ms */
	0xa4,			/* all-pixels-off */
	U8G_ESC_CS(0),		/* disable chip */
	U8G_ESC_END		/* end of sequence */
};

static const uint8_t u8g_dev_data_start[] = {
	U8G_ESC_ADR(0),		/* instruction mode */
	U8G_ESC_CS(1),		/* enable chip */
	0x10,			/* set upper 4 bit of the col adr to 0 */
	0x00,			/* set low 4 bits of the col adr to 0 */
	U8G_ESC_END		/* end of sequence */
};

static uint8_t
u8g_board_com_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr)
{
	switch (msg) {
	case U8G_COM_MSG_INIT:
		//debug("u8com: init");

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
			SPI_CR1_BAUDRATE_FPCLK_DIV_2,	/* XXX need to scope this - maybe able to go to 2 */
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
			//debug("u8com: data");
			/* display data */
			gpio_set(GPIOA, GPIO3);

		} else {
			//debug("u8com: command");
			/* display command */
			gpio_clear(GPIOA, GPIO3);
		}

		break;

	case U8G_COM_MSG_CHIP_SELECT:
		if (arg_val) {
			//debug("u8com: select");
			/* select display */
			gpio_clear(GPIOA, GPIO4);

		} else {
			//debug("u8com: deselect");
			/* deselect display */
			gpio_set(GPIOA, GPIO4);
		}

		break;

	case U8G_COM_MSG_RESET:
		if (arg_val) {
			//debug("u8com: clear reset");
			/* set reset line high */
			gpio_set(GPIOA, GPIO2);

		} else {
			//debug("u8com: assert reset");
			/* set reset line low */
			gpio_clear(GPIOA, GPIO2);
		}

		break;

	case U8G_COM_MSG_WRITE_BYTE:
		//debug("u8com: send 0x%02x", arg_val);
		spi_xfer(SPI1, arg_val);
		break;

	case U8G_COM_MSG_WRITE_SEQ:
	case U8G_COM_MSG_WRITE_SEQ_P:
		//debug("u8com: seq %d", arg_val);
		{
			uint8_t *ptr = (uint8_t *)arg_ptr;

			while (arg_val-- > 0) {
				//debug("u8com:   0x%02x", *ptr);
				spi_xfer(SPI1, *ptr++);
			}

			/* XXX wait for the last byte to finish transferring... */
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

		for (unsigned page = 0; page < (HEIGHT / PAGE_HEIGHT); page++) {
			u8g_WriteEscSeqP(u8g, dev, u8g_dev_data_start);
			u8g_WriteByte(u8g, dev, 0xb0 | page);
			u8g_SetAddress(u8g, dev, 1);
			for (unsigned column = 0; column < WIDTH; column++)
				u8g_WriteByte(u8g, dev, 0);
			u8g_SetChipSelect(u8g, dev, 0);			
		}
		break;

	case U8G_DEV_MSG_STOP:
		break;

	case U8G_DEV_MSG_PAGE_NEXT: {
			u8g_pb_t *pb = (u8g_pb_t *)(dev->dev_mem);

			u8g_WriteEscSeqP(u8g, dev, u8g_dev_data_start);
			u8g_WriteByte(u8g, dev, 0xb0 | pb->p.page);
			u8g_SetAddress(u8g, dev, 1);

			if (u8g_pb_WriteBuffer(pb, u8g, dev) == 0)
				return 0;

			u8g_SetChipSelect(u8g, dev, 0);
		}
		break;

	case U8G_DEV_MSG_CONTRAST:
		u8g_SetChipSelect(u8g, dev, 1);
		u8g_SetAddress(u8g, dev, 0);          /* instruction mode */
		u8g_WriteByte(u8g, dev, 0x81);
		u8g_WriteByte(u8g, dev, (*(uint8_t *)arg) >> 2);
		u8g_SetChipSelect(u8g, dev, 0);
		return 1;
	}

	return u8g_dev_pb8v1_base_fn(u8g, dev, msg, arg);
}

U8G_PB_DEV(u8g_board_dev, WIDTH, HEIGHT, PAGE_HEIGHT, u8g_board_dev_fn, u8g_board_com_fn);
