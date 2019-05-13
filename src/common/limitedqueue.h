#pragma once

#include <queue>

template <typename T>
class LimitedQueue : public std::queue<T>
{
public:
    using SuperClass = std::queue< T >;

    LimitedQueue();
    LimitedQueue( const unsigned int maxSize );

    void push( const T& value );

#if __cplusplus >= 201103L
    void push( T&& value );
#endif

    void setMaxSize( const unsigned int value );
    unsigned int maxSize() const;

protected:
    unsigned int m_maxSize;

    static const unsigned int m_defaultMaxSize = 10;

private:
    void initialize();

};

#include "limitedqueue.inl"
