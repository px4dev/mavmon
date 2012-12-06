
#include "board.h"

#include <u8g.h>

u8g_t u8g;

static void draw(void)
{
        u8g_SetFont(&u8g, u8g_font_6x10);
        u8g_DrawStr(&u8g, 0, 15, "Hello World!");
}

int main(void)
{
        int i;

        board_setup();

        u8g_Init(&u8g, &u8g_board_dev);

        do
        {
                draw();
        } while ( u8g_NextPage(&u8g) );

        while (1) {
                led_toggle();     /* LED on/off */
                for (i = 0; i < 1600000; i++)    /* Wait a bit. */
                        __asm__("nop");
                led_toggle();     /* LED on/off */
                for (i = 0; i <1600000; i++)    /* Wait a bit. */
                        __asm__("nop");
        }

        return 0;
}
