#include "tictoc.h"

#include <iostream>

// TicToc
TicToc::TicToc()
{
    tic();
}

void TicToc::tic()
{
    m_start = std::chrono::steady_clock::now();
}

double TicToc::toc()
{
    return std::chrono::duration< double >( std::chrono::steady_clock::now() - m_start ).count();
}

void TicToc::report()
{
    std::cout << "Elapsed time: " << toc() << " seconds" << std::endl;
}
