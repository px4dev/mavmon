
#include "board.h"

#include <u8g.h>
#include <scmRTOS.h>

/* initialiser list */
extern unsigned long __ctors_start__;
extern unsigned long __ctors_end__;

/* process list, must match scmRTOS_PROCESS_COUNT */
typedef OS::process<OS::pr0, 300> TProc1;
typedef OS::process<OS::pr1, 300> TProc2;

TProc1 Proc1;
TProc2 Proc2;

OS::TEventFlag TimerEvent;

u8g_t u8g;

static void draw(void)
{
	u8g_SetFont(&u8g, u8g_font_6x10);
	u8g_DrawStr(&u8g, 0, 15, "Hello World!");
}

int main(void)
{
	/* configure the board */
	board_setup();

	/* run constructors */
	for (unsigned long *ctors = &__ctors_start__; ctors < &__ctors_end__; )
	        ((void(*)(void))(*ctors++))();

	/* and start the OS */
	OS::run();
}

namespace OS
{
	template <>
	OS_PROCESS void TProc1::exec()
	{
		unsigned count = 0;
		for (;;) {
			if (count++ > 250) {
				led_toggle();     /* LED on/off */
				count = 0;
			}
			TimerEvent.wait();
		}
	}
	template <>
	OS_PROCESS void TProc2::exec()
	{
		u8g_Init(&u8g, &u8g_board_dev);

		do {
			draw();
		} while (u8g_NextPage(&u8g));

		for (;;) {
			sleep(100);
		}
	}
}

void OS::system_timer_user_hook()
{
	/* fire the timer event once per millisecond */
	TimerEvent.signal_isr();
}

