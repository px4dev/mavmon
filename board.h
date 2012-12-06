#include <u8g.h>

extern void	board_setup(void);
extern void	led_toggle(void);
extern void	serial_start(unsigned bitrate);
extern void	serial_stop(void);

extern u8g_dev_t u8g_board_dev;