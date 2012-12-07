#include <u8g.h>
#include <stdio.h>

#ifdef __cplusplus
# define EXTERN extern "C"
#else
# define EXTERN extern
#endif

#define debug(fmt, args...)	do { printf(fmt "\r\n", ##args); } while(0)

EXTERN void	board_setup(void);
EXTERN void	led_toggle(void);
EXTERN void	led_on(void);
EXTERN void	serial_start(unsigned bitrate);
EXTERN void	serial_stop(void);

EXTERN u8g_dev_t u8g_board_dev;
