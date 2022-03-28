#include <serial_device.h>

namespace Maylib
{
int USB;
std::mutex mtx_;  // 排他制御用ミューテックス
std::condition_variable cv_;
bool is_ready = false;  // for spurious wakeup
}  // namespace Maylib

using namespace Maylib;

char SerialDevice::receive_buffer[256];

void signal_handler_IO(int status); /* definition of signal handler */

SerialDevice::SerialDevice(/* args */)
{
}

SerialDevice::~SerialDevice()
{
}

bool SerialDevice::open(const char* port_name_, int baud_rate_, int time_out_)
{
  USB = ::open(port_name_, O_RDWR | O_NOCTTY);
  baud_rate = baud_rate_;
  time_out = time_out_;
}

void SerialDevice::startSerialThread()
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
  tty.c_cc[VTIME] = time_out;     // 0.5 seconds read timeout
  tty.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines

  /* Make raw */
  cfmakeraw(&tty);

  /* Flush Port, then applies attributes */
  tcflush(USB, TCIFLUSH);
  if (tcsetattr(USB, TCSANOW, &tty) != 0)
  {
    std::cout << "Error " << errno << " from tcsetattr" << std::endl;
  }

  try
  {
    // static std::thread th_a(ThreadWrite);
    static std::thread th_b(ThreadRead);
    // th_a.join();
    // th_b.join();
    is_ready = true;
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << std::endl;
  }
}

// void SerialDevice::ThreadWrite()
// {
//   for (size_t i = 0; i < 2; i++)
//   {
//     unsigned char cmd[] = "INIT\n";
//     int n_written = 0, spot = 0;
//     printf("send...\n");
//     do
//     {
//       std::lock_guard<std::mutex> lock(mtx_);
//       n_written = ::write(USB, &cmd[spot], 1);
//       spot += n_written;
//     } while (cmd[spot - 1] != '\n' && n_written > 0);

//     std::this_thread::sleep_for(std::chrono::seconds(1));
//   }
// }

void SerialDevice::ThreadRead()
{
  char buffer[256];

  while (1)
  {
    std::unique_lock<std::mutex> uniq_lk(mtx_);
    cv_.wait(uniq_lk, [] { return is_ready; });

    printf("receive...\n");

    memset(buffer, '\0', sizeof(buffer));

    int k = 0;

    k = read(USB, buffer, sizeof(buffer));
    printf("receive char : %d %s \n", k, buffer);

    memset(receive_buffer, '\0', sizeof(receive_buffer));
    memcpy(receive_buffer, buffer, sizeof(buffer));

    is_ready = false;
  }
}

void SerialDevice::write(unsigned char* req_)
{
  int n_written = 0, spot = 0;
  printf("send...\n");
  is_ready = false;
  do
  {
    std::lock_guard<std::mutex> lock(mtx_);
    // std::unique_lock<std::mutex> uniq_lk(mtx_);
    n_written = ::write(USB, &req_[spot], 1);
    spot += n_written;
  } while (req_[spot - 1] != '\n' && n_written > 0);
  is_ready = true;
}

void signal_handler_IO(int status)
{
  printf("received data from UART.\n");
  std::lock_guard<std::mutex> lock(mtx_);
  // is_ready = true;
  cv_.notify_one();
}