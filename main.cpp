#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <sys/signal.h>
#include <errno.h>
#include <termios.h>
/* https://stackoverflow.com/questions/15119412/setting-serial-port-interruption-in-linux */

void signal_handler_IO(int status); /* definition of signal handler */

int n;
int fd;
int connected;
struct termios termAttr;
struct sigaction saio;

int main(int argc, char* argv[])
{
  fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd == -1)
  {
    perror("open_port: Unable to open /dev/ttyO1\n");
    exit(1);
  }

  saio.sa_handler = signal_handler_IO;
  saio.sa_flags = 0;
  saio.sa_restorer = NULL;
  sigaction(SIGIO, &saio, NULL);

  fcntl(fd, F_SETFL, FNDELAY);
  fcntl(fd, F_SETOWN, getpid());
  fcntl(fd, F_SETFL, O_ASYNC);

  tcgetattr(fd, &termAttr);
  cfsetispeed(&termAttr, B115200);
  cfsetospeed(&termAttr, B115200);
  termAttr.c_cflag &= ~PARENB;
  termAttr.c_cflag &= ~CSTOPB;
  termAttr.c_cflag &= ~CSIZE;
  termAttr.c_cflag |= CS8;
  termAttr.c_cflag |= (CLOCAL | CREAD);
  termAttr.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  termAttr.c_iflag &= ~(IXON | IXOFF | IXANY);
  termAttr.c_oflag &= ~OPOST;
  tcsetattr(fd, TCSANOW, &termAttr);
  printf("UART1 configured....\n");

  connected = 1;
  while (connected == 1)
  {
    // write(fd, "Waiting for data...\n", 20);
    printf("Waiting for data...\n");
    sleep(1);
  }

  close(fd);
  exit(0);
}

int spot = 0;
char buf = '\0';

/* Whole response*/
char response[256];
unsigned char buffer[256];

void signal_handler_IO(int status)
{
  printf("received data from UART.\n");
  int len;
  do
  {
    len = read(fd, response, 256);

    printf("%s", response);
  } while (len > 0);

  // do
  // {
  //   n = read(fd, &buf, 1);
  //   sprintf(&response[spot], "%c", buf);
  //   spot += n;
  // } while (buf != '\r' && n > 0);

  // if (n < 0)
  // {
  //   std::cout << "Error reading: " << strerror(errno) << std::endl;
  // }
  // else if (n == 0)
  // {
  //   std::cout << "Read nothing!" << std::endl;
  // }
  // else
  // {
  //   std::cout << "Response: " << response << std::endl;
  // }

  // n = spot = 0;
  // /// zero padding response
  // for (int i = 0; i < 256; i++)
  // {
  //   response[i] = '\0';
  // }
}