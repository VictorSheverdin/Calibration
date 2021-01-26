template < class T >
Parent_Shared_Ptr< T >::Parent_Shared_Ptr( const Ptr &value )
{
    setParentPointer( value );
}

template < class T >
void Parent_Shared_Ptr< T >::setParentPointer( const Ptr &value )
{
    m_parentPointer = value;
}

template < class T >
typename Parent_Shared_Ptr< T >::Ptr Parent_Shared_Ptr< T >::parentPointer() const
{
    return m_parentPointer.lock();
}

