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
 * @file board.h
 *
 * Board interface definitions.
 */

#pragma once

#include <u8g.h>
#include <m2.h>

#include "mavmon.h"

/*
 * The board support code must export the u8g driver and m2 event sources.
 */
EXTERN u8g_dev_t u8g_board_dev;
EXTERN uint8_t m2_board_es(m2_p ep, uint8_t msg);

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

	/**
	 * Called by the board-specific subclass to add a byte
	 * to the receive buffer.
	 *
	 * @param c		The received byte.
	 */
	void			com_rx(uint8_t c);

	/**
	 * Called by the board-specific subclass when it wants a
	 * byte to transmit.
	 *
	 * @param c		The byte to send.
	 * @return		True if a byte was returned.
	 */
	bool			com_tx(uint8_t &c);

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
