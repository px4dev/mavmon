
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
