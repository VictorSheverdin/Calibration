#pragma once

#include <vector>
#include <stdexcept>

template < class TYPE >
class matrix
{
public:
    matrix();
    matrix( const size_t rows, const size_t cols );

    void create( const size_t rows, const size_t cols );

    const TYPE &at( const size_t row, const size_t col ) const;
    TYPE &at( const size_t row, const size_t col );

protected:
    std::vector< TYPE > m_data;

    size_t m_rows;
    size_t m_cols;
};

#include "matrix.inl"
