#ifndef _uart901_h_
#define _uart901_h_

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <assert.h>
#include <termios.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <sys/types.h>
#include <errno.h>

int jy901_init(const char *port_str);
int jy901_loop(void);
float jy901_getGyro(int axis);
float jy901_getAcc(int axis);
float jy901_getEuler(int axis);
int jy901_close(void);

#endif
