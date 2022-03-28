#ifndef SERIAL_DEVICE_H
#define SERIAL_DEVICE_H

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

#include <map>

class SerialDevice
{
private:
  // static void ThreadWrite();
  static void ThreadRead();

  int time_out;
  std::map<int, int> baud_rate_map = {
    { 0, B0 },         { 50, B50 },       { 75, B75 },         { 110, B110 },       { 134, B134 },
    { 150, B150 },     { 200, B200 },     { 300, B300 },       { 600, B600 },       { 1200, B1200 },
    { 1800, B1800 },   { 2400, B2400 },   { 4800, B4800 },     { 9600, B9600 },     { 19200, B19200 },
    { 38400, B38400 }, { 57600, B57600 }, { 115200, B115200 }, { 230400, B230400 },
  };
  int baud_rate;

public:
  SerialDevice(/* args */);
  ~SerialDevice();
  bool open(const char* port_name_, int baud_rate_, int time_out_);
  void startSerialThread();
  void write(unsigned char* req_);

  static char receive_buffer[256];
};

#endif  // SERIAL_DEVICE_H