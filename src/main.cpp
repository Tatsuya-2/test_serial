#include <serial_device.h>

int main()
{
  SerialDevice serial_device;
  serial_device.open("/dev/ttyACM0", 115200, 1);
  serial_device.startSerialThread();

  unsigned char cmd[] = "INIT\n";

  while (1)
  {
    serial_device.write(cmd);
    std::this_thread::sleep_for(std::chrono::milliseconds(33));
  }

  return 0;
}