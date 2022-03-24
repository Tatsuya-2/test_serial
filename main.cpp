#include <cstdint>
#include <iostream>
#include <mutex>
#include <condition_variable>
#include <thread>
/* https://qiita.com/nsnonsugar/items/be8a066c6627ab5b052a */
/* https://qiita.com/termoshtt/items/c01745ea4bcc89d37edc */

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

int USB = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);

std::mutex mtx_;  // 排他制御用ミューテックス
std::condition_variable cv_;
bool is_ready = false;  // for spurious wakeup

uint32_t count_;
int condition;

void signal_handler_IO(int status); /* definition of signal handler */

void add_count()
{
  // count_を加算する前にミューテックスを取得する
  std::lock_guard<std::mutex> lock(mtx_);
  ++count_;
}

void ThreadWrite()
{
  for (size_t i = 0; i < 20; i++)
  {
    unsigned char cmd[] = "INIT\n";
    int n_written = 0, spot = 0;
    printf("send...\n");
    do
    {
      std::lock_guard<std::mutex> lock(mtx_);
      n_written = write(USB, &cmd[spot], 1);
      spot += n_written;
    } while (cmd[spot - 1] != '\n' && n_written > 0);

    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

void ThreadRead()
{
  // pthread_mutex_init(&mutex0, NULL);
  // pthread_cond_init(&cond0, NULL);

  // memset(&action, 0, sizeof(action));
  // memset(&evp, 0, sizeof(evp));

  char buffer[256];

  while (1)
  {
    std::unique_lock<std::mutex> uniq_lk(mtx_);  // ここでロックされる
    cv_.wait(uniq_lk, [] { return is_ready; });

    printf("receive...\n");

    // while (!condition)
    // {
    //   pthread_cond_wait(&cond0, &mutex0);
    // }

    // condition = 0;

    memset(buffer, '\0', sizeof(buffer));

    int k = 0;

    k = read(USB, buffer, sizeof(buffer));
    printf("receive char : %d %s \n", k, buffer);

    is_ready = false;
  }

  pthread_exit(0);
}

int main()
{
  struct termios tty;
  struct termios tty_old;
  struct sigaction saio;
  memset(&tty, 0, sizeof tty);

  /* Error Handling */
  if (tcgetattr(USB, &tty) != 0)
  {
    std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
  }

  /* Save old tty parameters */
  tty_old = tty;

  saio.sa_handler = signal_handler_IO;
  saio.sa_flags = 0;
  saio.sa_restorer = NULL;
  sigaction(SIGIO, &saio, NULL);

  fcntl(USB, F_SETFL, FNDELAY);
  fcntl(USB, F_SETOWN, getpid());
  fcntl(USB, F_SETFL, O_ASYNC);

  tcgetattr(USB, &tty);

  /* Set Baud Rate */
  cfsetospeed(&tty, (speed_t)B115200);
  cfsetispeed(&tty, (speed_t)B115200);

  /* Setting other Port Stuff */
  tty.c_cflag &= ~PARENB;  // Make 8n1
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;

  tty.c_cflag &= ~CRTSCTS;        // no flow control
  tty.c_cc[VMIN] = 1;             // read doesn't block
  tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout
  tty.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines

  /* Make raw */
  cfmakeraw(&tty);

  /* Flush Port, then applies attributes */
  tcflush(USB, TCIFLUSH);
  if (tcsetattr(USB, TCSANOW, &tty) != 0)
  {
    std::cout << "Error " << errno << " from tcsetattr" << std::endl;
  }

  count_ = 0;

  std::thread th_a(ThreadWrite);
  std::thread th_b(ThreadRead);

  th_a.join();
  th_b.join();

  // std::cout << "count_ : " << count_ << std::endl;

  return 0;
}

void signal_handler_IO(int status)
{
  printf("received data from UART.\n");
  std::lock_guard<std::mutex> lock(mtx_);
  is_ready = true;
  cv_.notify_one();
}