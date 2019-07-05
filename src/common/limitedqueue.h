#pragma once

#include <list>

template <typename T>
class LimitedQueue
{
public:
    LimitedQueue();
    LimitedQueue( const unsigned int maxSize );

    void push( const T& value );

#if __cplusplus >= 201103L
    void push( T&& value );
#endif

    void pop();

    T &front();
    const T &front() const;

    T &back();
    const T &back() const;

    size_t size() const;
    bool empty() const;

    typename std::list<T>::iterator begin();
    typename std::list<T>::const_iterator begin() const;

    typename std::list<T>::reverse_iterator rbegin();
    typename std::list<T>::const_reverse_iterator rbegin() const;

    typename std::list<T>::iterator end();
    typename std::list<T>::const_iterator end() const;

    typename std::list<T>::reverse_iterator rend();
    typename std::list<T>::const_reverse_iterator rend() const;

    void setMaxSize( const unsigned int value );
    unsigned int maxSize() const;

protected:
    std::list<T> m_list;
    unsigned int m_maxSize;

    static const unsigned int m_defaultMaxSize = 20;

private:
    void initialize();

};

#include "limitedqueue.inl"
