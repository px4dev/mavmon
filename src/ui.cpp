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
 * @file ui.cpp
 *
 * User interface for MAVmon.
 */

#include <u8g.h>
#include <m2.h>
//#include <m2utl.h> /* doesn't seem necessary yet */
#include <m2ghu8g.h>

#include "mavmon.h"
#include "board.h"
#include "ui.h"

static u8g_t u8g;

typedef OS::process<OS_PRIO_GUI, 1024> TUIProc;
TUIProc UIProc;



namespace OS
{
template <>
OS_PROCESS void TUIProc::exec()
{
	/* do u8g init */
	u8g_Init(&u8g, &u8g_board_dev);

	/* do m2 init */
	m2_Init(&_top, 			/* UI root */
		m2_board_es,		/* event source */
		m2_eh_4bd,		/* event handler */
		m2_gh_u8g_bfs);		/* UI style */
	m2_SetU8g(&u8g, m2_u8g_box_icon);
	m2_SetFont(0, (const void *)u8g_font_04b_03r);
	m2_SetFont(1, (const void *)u8g_font_6x13);
	m2_SetFont(2, (const void *)u8g_font_m2icon_7);

	for (;;) {
		m2_CheckKey();

		if (m2_HandleKey()) {
			/* picture loop */
			u8g_FirstPage(&u8g);

			do {
				m2_Draw();
				m2_CheckKey();
			} while (u8g_NextPage(&u8g));
		}

		sleep(10);
	}
}
}

/*
 * u8g delay function
 */
extern "C" void
u8g_Delay(uint16_t val)
{
	OS::sleep(val);
}
