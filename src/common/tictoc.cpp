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
    return static_cast< double>( std::chrono::duration_cast< std::chrono::milliseconds >( std::chrono::steady_clock::now() - m_start ).count() ) * 1.e-3;
}

void TicToc::report()
{
    std::cout << "Elapsed time: " << toc() << " seconds" << std::endl;
}
