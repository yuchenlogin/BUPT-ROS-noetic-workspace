#ifndef _keyboard_h_
#define _keyboard_h_

#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <termios.h>
#include <unistd.h>

int tty_reset(void);
int tty_set(void);
int kbhit(void);

#endif
