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

    m_list.push_back( value );

}

#if __cplusplus >= 201103L

    template < typename T >
    void LimitedQueue< T >::push( T &&value )
    {
        while ( this->size() >= m_maxSize )
            this->pop();

        m_list.push_back( value );

    }

#endif

template < typename T >
void LimitedQueue<T>::pop()
{
    m_list.pop_front();
}

template < typename T >
T &LimitedQueue<T>::front()
{
    return m_list.front();
}

template < typename T >
const T &LimitedQueue<T>::front() const
{
    return m_list.front();
}

template < typename T >
T &LimitedQueue<T>::back()
{
    return m_list.back();
}

template < typename T >
const T &LimitedQueue<T>::back() const
{
    return m_list.back();
}

template < typename T >
size_t LimitedQueue<T>::size() const
{
    return m_list.size();
}

template < typename T >
bool LimitedQueue<T>::empty() const
{
    return m_list.empty();
}

template < typename T >
typename std::list<T>::iterator LimitedQueue<T>::begin()
{
    return m_list.begin();
}

template < typename T >
typename std::list<T>::const_iterator LimitedQueue<T>::begin() const
{
    return m_list.begin();
}

template < typename T >
typename std::list<T>::reverse_iterator LimitedQueue<T>::rbegin()
{
    return m_list.rbegin();
}

template < typename T >
typename std::list<T>::const_reverse_iterator LimitedQueue<T>::rbegin() const
{
    return m_list.rbegin();
}

template < typename T >
typename std::list<T>::iterator LimitedQueue<T>::end()
{
    return m_list.end();
}

template < typename T >
typename std::list<T>::const_iterator LimitedQueue<T>::end() const
{
    return m_list.end();
}

template < typename T >
typename std::list<T>::reverse_iterator LimitedQueue<T>::rend()
{
    return m_list.rend();
}

template < typename T >
typename std::list<T>::const_reverse_iterator LimitedQueue<T>::rend() const
{
    return m_list.rend();
}

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
