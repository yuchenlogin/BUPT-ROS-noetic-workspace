#include "uartjy901.h"
#include<iostream>

static int fd;
static char r_buf[1024];

static float a[3], w[3], Angle[3], h[3];

#define BAUD 115200 //115200 for JY61 ,9600 for others
#define DEBUG_PRINT 0

int uart_open(int fd, const char *pathname)
{
  fd = open(pathname, O_RDWR | O_NOCTTY);
  if (-1 == fd)
  {
    perror("Can't Open Serial Port");
    return (-1);
  }
  else
    printf("open %s success!\n", pathname);
  if (isatty(STDIN_FILENO) == 0)
    printf("standard input is not a terminal device\n");
  else
    printf("isatty success!\n");
  return fd;
}

int uart_set(int fd, int nSpeed, int nBits, char nEvent, int nStop)
{
  struct termios newtio, oldtio;
  if (tcgetattr(fd, &oldtio) != 0)
  {
    perror("SetupSerial 1");
    printf("tcgetattr( fd,&oldtio) -> %d\n", tcgetattr(fd, &oldtio));
    return -1;
  }
  bzero(&newtio, sizeof(newtio));
  newtio.c_cflag |= CLOCAL | CREAD;
  newtio.c_cflag &= ~CSIZE;
  switch (nBits)
  {
  case 7:
    newtio.c_cflag |= CS7;
    break;
  case 8:
    newtio.c_cflag |= CS8;
    break;
  }
  switch (nEvent)
  {
  case 'o':
  case 'O':
    newtio.c_cflag |= PARENB;
    newtio.c_cflag |= PARODD;
    newtio.c_iflag |= (INPCK | ISTRIP);
    break;
  case 'e':
  case 'E':
    newtio.c_iflag |= (INPCK | ISTRIP);
    newtio.c_cflag |= PARENB;
    newtio.c_cflag &= ~PARODD;
    break;
  case 'n':
  case 'N':
    newtio.c_cflag &= ~PARENB;
    break;
  default:
    break;
  }

  /*设置波特率*/

  switch (nSpeed)
  {
  case 2400:
    cfsetispeed(&newtio, B2400);
    cfsetospeed(&newtio, B2400);
    break;
  case 4800:
    cfsetispeed(&newtio, B4800);
    cfsetospeed(&newtio, B4800);
    break;
  case 9600:
    cfsetispeed(&newtio, B9600);
    cfsetospeed(&newtio, B9600);
    break;
  case 115200:
    cfsetispeed(&newtio, B115200);
    cfsetospeed(&newtio, B115200);
    break;
  case 460800:
    cfsetispeed(&newtio, B460800);
    cfsetospeed(&newtio, B460800);
    break;
  default:
    cfsetispeed(&newtio, B9600);
    cfsetospeed(&newtio, B9600);
    break;
  }
  if (nStop == 1)
    newtio.c_cflag &= ~CSTOPB;
  else if (nStop == 2)
    newtio.c_cflag |= CSTOPB;
  newtio.c_cc[VTIME] = 0;
  newtio.c_cc[VMIN] = 0;
  tcflush(fd, TCIFLUSH);

  if ((tcsetattr(fd, TCSANOW, &newtio)) != 0)
  {
    perror("com set error");
    return -1;
  }
  printf("set done!\n");
  return 0;
}

int uart_close(int fd)
{
  assert(fd);
  close(fd);

  return 0;
}

int send_data(int fd, char *send_buffer, int length)
{
  length = write(fd, send_buffer, length * sizeof(unsigned char));
  return length;
}

int recv_data(int fd, char *recv_buffer, int length)
{
  length = read(fd, recv_buffer, length);
  return length;
}

void ParseData(char chr)
{
  static char chrBuf[100];
  static unsigned char chrCnt = 0;
  signed short sData[4];
  unsigned char i;

  time_t now;
  chrBuf[chrCnt++] = chr;
  if (chrCnt < 11)
    return;

  if ((chrBuf[0] != 0x55) || ((chrBuf[1] & 0x50) != 0x50))
  {
    printf("ParseData Error:%x %x\r\n", chrBuf[0], chrBuf[1]);
    memcpy(&chrBuf[0], &chrBuf[1], 10);
    chrCnt--;
    return;
  }

  memcpy(&sData[0], &chrBuf[2], 8);
  switch (chrBuf[1])
  {
  case 0x51:
    for (i = 0; i < 3; i++)
      a[i] = (float)sData[i] / 32768.0 * 16.0;
    break;
  case 0x52:
    for (i = 0; i < 3; i++)
      w[i] = (float)sData[i] / 32768.0 * 2000.0;
    break;
  case 0x53:
    for (i = 0; i < 3; i++)
      Angle[i] = (float)sData[i] / 32768.0 * 180.0;
    break;
  case 0x54:
    for (i = 0; i < 3; i++)
      h[i] = (float)sData[i];
    break;
  }

#if DEBUG_PRINT
  time(&now);
  printf("\r\nT:%s a:%6.3f %6.3f %6.3f ", asctime(localtime(&now)), a[0], a[1], a[2]);
  printf("w:%7.3f %7.3f %7.3f ", w[0], w[1], w[2]);
  printf("A:%7.3f %7.3f %7.3f ", Angle[0], Angle[1], Angle[2]);
  printf("h:%4.0f %4.0f %4.0f ", h[0], h[1], h[2]);
#endif

  chrCnt = 0;
}

int jy901_init(const char *port_str)
{
  bzero(r_buf, 1024);

  fd = uart_open(fd, port_str); /*串口号 /dev/ttySn,USB口号 /dev/ttyUSBn */
  if (fd == -1)
  {
    fprintf(stderr, "uart_open error\n");
    return -1;
  }

  if (uart_set(fd, BAUD, 8, 'N', 1) == -1)
  {
    fprintf(stderr, "uart set failed!\n");
    return -1;
  }
  return 0;
}

int jy901_loop()
{
  int ret = 0;

  ret = recv_data(fd, r_buf, 44);
  if (ret == -1)
  {
    fprintf(stderr, "uart read failed!");
    return -1;
  }
  if (ret == 0)
  {
    return -2;
  }
  for (int i = 0; i < ret; i++)
  {
    ParseData(r_buf[i]);
  }
  usleep(1000);
  return 0;
}

float jy901_getGyro(int axis)
{
  if (axis < 0 || axis > 2)
  {
    fprintf(stderr, "jy901_getGyro param error!\n");
    return 0;
  }
  return w[axis];
}

float jy901_getAcc(int axis)
{
  if (axis < 0 || axis > 2)
  {
    fprintf(stderr, "jy901_getAcc param error!\n");
    return 0;
  }
  return a[axis];
}

float jy901_getEuler(int axis)
{
  if (axis < 0 || axis > 2)
  {
    fprintf(stderr, "jy901_getEuler param error!\n");
    return 0;
  }
  return Angle[axis];
}

int jy901_close()
{
  int ret = 0;

  ret = uart_close(fd);
  if (ret == -1)
  {
    fprintf(stderr, "uart_close error\n");
    return -1;
  }

  return 0;
}

// int main(void)
// {
//     int ret;
//     char r_buf[1024];
//     bzero(r_buf, 1024);

//     fd = uart_open(fd, "/dev/ttyUSB0"); /*串口号/dev/ttySn,USB口号/dev/ttyUSBn */
//     if (fd == -1)
//     {
//         fprintf(stderr, "uart_open error\n");
//         exit(EXIT_FAILURE);
//     }

//     if (uart_set(fd, BAUD, 8, 'N', 1) == -1)
//     {
//         fprintf(stderr, "uart set failed!\n");
//         exit(EXIT_FAILURE);
//     }

//     FILE *fp;
//     fp = fopen("Record.txt", "w");
//     while (1)
//     {
//         ret = recv_data(fd, r_buf, 44);
//         if (ret == -1)
//         {
//             fprintf(stderr, "uart read failed!\n");
//             exit(EXIT_FAILURE);
//         }
//         for (int i = 0; i < ret; i++)
//         {
//             fprintf(fp, "%2X ", r_buf[i]);
//             ParseData(r_buf[i]);
//         }
//         usleep(1000);
//     }

//     ret = uart_close(fd);
//     if (ret == -1)
//     {
//         fprintf(stderr, "uart_close error\n");
//         exit(EXIT_FAILURE);
//     }

//     exit(EXIT_SUCCESS);
// }
