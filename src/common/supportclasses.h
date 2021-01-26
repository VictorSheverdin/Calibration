#pragma once

template < class T >
class Parent_Shared_Ptr
{
public:
    using Ptr = std::shared_ptr< T >;

    Parent_Shared_Ptr() = default;
    Parent_Shared_Ptr( const Ptr &value );

    void setParentPointer( const Ptr &value );
    Ptr parentPointer() const;

protected:
    using PtrImpl = std::weak_ptr< T >;

    PtrImpl m_parentPointer;

};

#include "supportclasses.inl"
