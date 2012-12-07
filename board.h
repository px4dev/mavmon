
#pragma once

#include <u8g.h>
#include <stdio.h>
#include <scmRTOS.h>

#ifdef __cplusplus
# define EXTERN extern "C"
#else
# define EXTERN extern
#endif

#define debug(fmt, args...)	do { printf(fmt "\r\n", ##args); } while(0)

/**
 * The board driver must export a u8glib driver structure.
 */
EXTERN u8g_dev_t u8g_board_dev;

class RingBuffer;
class Board;

extern Board	*gBoard;

/**
 * Abstract class that board drivers inherit.
 */
class Board
{
public:
	Board(RingBuffer *com_tx_buf, RingBuffer *com_rx_buf);

	/**
	 * Perform pre-constructor board initialisation.
	 */
	virtual void		setup(void);

	/**
	 * Configure the serial port.
	 *
	 * @param speed		The port speed in bits per second.
	 */
	virtual void		com_init(unsigned speed);

	/**
	 * Disable the serial port.
	 */
	virtual void		com_fini();

	/**
	 * Write data to the serial port.
	 *
	 * This function will block until the data has been queued for
	 * transmission.
	 *
	 * @param data		Pointer to the data to write.
	 * @param count		The number of bytes to write.
	 */
	void			com_write(const uint8_t *data, unsigned count);

	/**
	 * Read data from the serial port.
	 *
	 * @param data		Buffer into which data can be read.
	 * @param size		The size of the receive buffer.
	 * @return		The number of bytes read.
	 */
	int			com_read(uint8_t *data, unsigned size);

	/**
	 * Check the serial port transmit buffer space.
	 *
	 * @return		The number of bytes that can be written
	 *			without blocking.
	 */
	int			com_write_space();

	/**
	 * Check the serial port receive buffer.
	 *
	 * @return		The number of bytes available to read.
	 */
	int			com_read_available();

	/**
	 * Turn the LED on or off.
	 *
	 * @param state		The new LED state (true = on)
	 */
	virtual void		led_set(bool state);

	/**
	 * Toggle the LED.
	 */
	virtual void		led_toggle();


protected:
	RingBuffer		*_com_tx_buf;
	RingBuffer		*_com_rx_buf;

	OS::TEventFlag		_tx_space_avail;
	OS::TEventFlag		_rx_data_avail;

	/**
	 * Called by the generic code when bytes are added to the
	 * transmit buffer.
	 */
	virtual void		com_tx_start(void);

private:

};

/**
 * Simple ring buffer for serial etc. comms.
 */
class RingBuffer
{
public:
	RingBuffer(uint8_t *buffer, unsigned size) :
		_buffer(buffer),
		_size(size),
		_head(0),
		_tail(0) {
	}

	unsigned		capacity() { return _size; }
	unsigned		free();
	unsigned		contains();
	void			insert(uint8_t c);
	uint8_t			remove();

private:
	uint8_t			*_buffer;
	unsigned		_size;
	volatile unsigned	_head;
	volatile unsigned	_tail;

	unsigned		advance(unsigned index);
};
