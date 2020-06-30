#pragma once

#include <ctime>
#include <cstdlib>
#include <chrono>

class TicToc
{
  public:
    TicToc();

    void tic();
    double toc();

    void report();

  protected:
    std::chrono::time_point<std::chrono::system_clock> m_start;
};
