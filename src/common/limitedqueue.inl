// LimitedQueue
template < typename T >
LimitedQueue< T >::LimitedQueue()
{
    initialize();
}

template < typename T >
LimitedQueue< T >::LimitedQueue( const unsigned int maxSize )
{
    initialize();

    setMaxSize( maxSize );

}

template < typename T >
void LimitedQueue< T >::initialize()
{
    m_maxSize = m_defaultMaxSize;
}

template < typename T >
void LimitedQueue< T >::push( const T& value )
{

    while ( this->size() >= m_maxSize )
        this->pop();

    SuperClass::push( value );

}

#if __cplusplus >= 201103L

    template < typename T >
    void LimitedQueue< T >::push( T &&value )
    {
        while ( this->size() >= m_maxSize )
            this->pop();

        SuperClass::push( value );

    }

#endif

template < typename T >
void LimitedQueue< T >::setMaxSize( const unsigned int value )
{
    if ( value > 0 )
        m_maxSize = value;
}

template < typename T >
unsigned int LimitedQueue< T >::maxSize() const
{
    return m_maxSize;
}
