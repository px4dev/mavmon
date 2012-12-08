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

#include "board.h"

Board::Board(RingBuffer *com_tx_buf, RingBuffer *com_rx_buf) :
	_com_tx_buf(com_tx_buf),
	_com_rx_buf(com_rx_buf)
{}

/****************************************************************************
 * Serial port
 */

unsigned
RingBuffer::free()
{
	if (_head < _tail)
		return _tail - _head - 1;

	return _size - (_head - _tail) - 1;
}

unsigned
RingBuffer::contains()
{
	if (_head > _tail)
		return _head - _tail;

	return _head + (_size - _tail);
}

void
RingBuffer::insert(uint8_t c)
{
	unsigned next = advance(_head);

	if (next != _tail) {
		_buffer[_head] = c;
		_head = next;
	}
}

uint8_t
RingBuffer::remove()
{
	uint8_t c = 0;

	if (_tail != _head) {
		c = _buffer[_tail];
		_tail = advance(_tail);
	}

	return c;
}

unsigned
RingBuffer::advance(unsigned index)
{
	index++;

	if (index >= _size)
		return 0;

	return index;
}

void Board::setup(void) {}
void Board::com_init(unsigned speed) {}
void Board::com_fini() {}
void Board::com_tx_start(void) {}

void
Board::com_write(const uint8_t *data, unsigned count)
{
	TCritSect cs;

	while (count) {
		unsigned avail = _com_tx_buf->free();

		/* if we have no tx space, wait for some to free up */
		if (avail == 0) {
			_tx_space_avail.wait();
			continue;
		}

		if (avail > count)
			avail = count;

		while (avail--)
			_com_tx_buf->insert(*data++);

		count -= avail;
		com_tx_start();
	}
}

int
Board::com_read(uint8_t *data, unsigned size)
{
	TCritSect cs;

	unsigned avail = _com_rx_buf->contains();

	if (avail > size)
		avail = size;

	for (unsigned i = 0; i < avail; i++)
		data[i] = _com_rx_buf->remove();

	return avail;
}

int
Board::com_write_space()
{
	return _com_tx_buf->free();
}

int
Board::com_read_available()
{
	return _com_rx_buf->contains();
}

void Board::led_set(bool state) {}
void Board::led_toggle() {}
