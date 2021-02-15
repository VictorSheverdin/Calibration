template < class T >
Parent_Weak_Ptr< T >::Parent_Weak_Ptr( const Ptr &value )
{
    setParentPointer( value );
}

template < class T >
void Parent_Weak_Ptr< T >::setParentPointer( const Ptr &value )
{
    m_parentPointer = value;
}

template < class T >
typename Parent_Weak_Ptr< T >::Ptr Parent_Weak_Ptr< T >::parentPointer() const
{
    return m_parentPointer.lock();
}

