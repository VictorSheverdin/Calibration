#pragma once

template < class T >
class Parent_Weak_Ptr
{
public:
    using Ptr = std::shared_ptr< T >;

    Parent_Weak_Ptr() = default;
    Parent_Weak_Ptr( const Ptr &value );

    void setParentPointer( const Ptr &value );
    Ptr parentPointer() const;

protected:
    using PtrImpl = std::weak_ptr< T >;

    PtrImpl m_parentPointer;

};

#include "supportclasses.inl"
