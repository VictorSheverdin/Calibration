template < class TYPE >
matrix< TYPE >::matrix()
    : m_rows( 0 ), m_cols( 0 )
{
}

template < class TYPE >
matrix< TYPE >::matrix( const size_t rows, const size_t cols )
{
    create( rows, cols );
}

template < class TYPE >
void matrix< TYPE >::create( const size_t rows, const size_t cols )
{
    m_data = std::vector< TYPE >( rows * cols );
    m_rows = rows;
    m_cols = cols;
}

template < class TYPE >
const TYPE &matrix< TYPE >::at( const size_t row, const size_t col ) const
{
    if ( row >= m_rows || col >= m_cols )
        throw std::out_of_range( "Matrix index is out of range." );

    return m_data.at( row * m_cols + col );
}

template < class TYPE >
TYPE &matrix< TYPE >::at( const size_t row, const size_t col )
{
    if ( row >= m_rows || col >= m_cols )
        throw std::out_of_range( "Matrix index is out of range." );

    return m_data.at( row * m_cols + col );
}
