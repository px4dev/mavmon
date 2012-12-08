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
 * @file main.cpp
 *
 * Application startup logic.
 */

#include "mavmon.h"
#include "board.h"

/* initialiser list */
extern unsigned long __ctors_start__;
extern unsigned long __ctors_end__;

extern "C" int
main(void)
{
	/* run constructors first */
	for (unsigned long *ctors = &__ctors_start__; ctors < &__ctors_end__;)
		((void( *)(void))(*ctors++))();

	/* configure the board */
	gBoard->setup();

	/* XXX debugging */
	gBoard->com_init(115200);
	debug("mavmon");


	/* and start the OS */
	OS::run();
}

typedef OS::process<OS_PRIO_LED, 200> TLEDProc;
TLEDProc LEDProc;

namespace OS
{
TEventFlag TimerEvent;

template <>
OS_PROCESS void TLEDProc::exec()
{
	unsigned count = 0;

	for (;;) {
		if (count++ > 125) {
			gBoard->led_toggle();     /* LED on/off */
			count = 0;
		}

		TimerEvent.wait();
	}
}

void
system_timer_user_hook()
{
	/* fire the timer event once per millisecond */
	TimerEvent.signal_isr();
}

}