
#include "board.h"

#include <u8g.h>
#include <scmRTOS.h>

#include <stdio.h>

/* initialiser list */
extern unsigned long __ctors_start__;
extern unsigned long __ctors_end__;

/* process list, must match scmRTOS_PROCESS_COUNT */
typedef OS::process<OS::pr0, 1024> TProc1;
typedef OS::process<OS::pr1, 1024> TProc2;

TProc1 Proc1;
TProc2 Proc2;

OS::TEventFlag TimerEvent;

u8g_t u8g;

void u8g_prepare(void) {
  u8g_SetFont(&u8g, u8g_font_6x10);
  u8g_SetFontRefHeightExtendedText(&u8g);
  u8g_SetDefaultForegroundColor(&u8g);
  u8g_SetFontPosTop(&u8g);
}

void u8g_box_frame(uint8_t a) {
  u8g_DrawStr(&u8g, 0, 0, "drawBox");
  u8g_DrawBox(&u8g, 5,10,20,10);
  u8g_DrawBox(&u8g, 10+a,15,30,7);
  u8g_DrawStr(&u8g, 0, 30, "drawFrame");
  u8g_DrawFrame(&u8g, 5,10+30,20,10);
  u8g_DrawFrame(&u8g, 10+a,15+30,30,7);
}

void u8g_string(uint8_t a) {
  u8g_DrawStr(&u8g, 30+a,31, " 0");
  u8g_DrawStr90(&u8g, 30,31+a, " 90");
  u8g_DrawStr180(&u8g, 30-a,31, " 180");
  u8g_DrawStr270(&u8g, 30,31-a, " 270");
}

void u8g_line(uint8_t a) {
  u8g_DrawStr(&u8g, 0, 0, "drawLine");
  u8g_DrawLine(&u8g, 7+a, 10, 40, 55);
  u8g_DrawLine(&u8g, 7+a*2, 10, 60, 55);
  u8g_DrawLine(&u8g, 7+a*3, 10, 80, 55);
  u8g_DrawLine(&u8g, 7+a*4, 10, 100, 55);
}

void u8g_ascii_1(void) {
  char s[2] = " ";
  uint8_t x, y;
  u8g_DrawStr(&u8g, 0, 0, "ASCII page 1");
  for( y = 0; y < 6; y++ ) {
    for( x = 0; x < 16; x++ ) {
      s[0] = y*16 + x + 32;
      u8g_DrawStr(&u8g, x*7, y*10+10, s);
    }
  }
}

void u8g_ascii_2(void) {
  char s[2] = " ";
  uint8_t x, y;
  u8g_DrawStr(&u8g, 0, 0, "ASCII page 2");
  for( y = 0; y < 6; y++ ) {
    for( x = 0; x < 16; x++ ) {
      s[0] = y*16 + x + 160;
      u8g_DrawStr(&u8g, x*7, y*10+10, s);
    }
  }
}


uint8_t draw_state = 0;

void draw(void) {
  u8g_prepare();
  switch(draw_state >> 3) {
    case 0: u8g_box_frame(draw_state&7); break;
    case 1: u8g_string(draw_state&7); break;
    case 2: u8g_line(draw_state&7); break;
    case 3: u8g_ascii_1(); break;
    case 4: u8g_ascii_2(); break;
  }
}


int 
main(void)
{

	/* run constructors first */
	for (unsigned long *ctors = &__ctors_start__; ctors < &__ctors_end__; )
	        ((void(*)(void))(*ctors++))();

	/* configure the board */
	gBoard->setup();

	/* XXX debugging */
	gBoard->com_init(115200);
	debug("mavmon");


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
			if (count++ > 125) {
				gBoard->led_toggle();     /* LED on/off */
				count = 0;
			}
			TimerEvent.wait();
		}
	}

	template <>
	OS_PROCESS void TProc2::exec()
	{
		u8g_Init(&u8g, &u8g_board_dev);
		u8g_SetRot180(&u8g);
		u8g_SetColorIndex(&u8g, 1);		/* pixel on */

		for (;;) {
			u8g_FirstPage(&u8g);
			do {
				draw();
			} while ( u8g_NextPage(&u8g) );
			
			draw_state++;
			if ( draw_state >= 5*8 )
				draw_state = 0;			

			sleep(100);
		}
	}
}

void OS::system_timer_user_hook()
{
	/* fire the timer event once per millisecond */
	TimerEvent.signal_isr();
}

/*
 * u8g delay functions
 */
extern "C" void
u8g_Delay(uint16_t val)
{
	OS::sleep(val);
}
