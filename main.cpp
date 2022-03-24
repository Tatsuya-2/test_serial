#include <cstdint>
#include <iostream>
#include <mutex>
#include <thread>

std::mutex mtx_;  // 排他制御用ミューテックス
uint32_t count_;

void add_count()
{
  // count_を加算する前にミューテックスを取得する
  std::lock_guard<std::mutex> lock(mtx_);
  ++count_;
}

void ThreadA()
{
  for (int i = 0; i < 100000; ++i)
  {
    add_count();
  }
}

void ThreadB()
{
  for (int i = 0; i < 100000; ++i)
  {
    add_count();
  }
}

int main()
{
  count_ = 0;

  std::thread th_a(ThreadA);
  std::thread th_b(ThreadB);

  th_a.join();
  th_b.join();

  std::cout << "count_ : " << count_ << std::endl;

  return 0;
}