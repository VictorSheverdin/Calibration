#include "precompiled.h"

#include "plane.h"

#include "defs.h"

// Plane
Plane::Plane()
{
    initialize();
}

Plane::Plane( const double a, const double b, const double c, const double d )
{
    initialize();

    set( a, b, c, d );
}

void Plane::initialize()
{
    m_a = m_b = m_c = m_d = 0.0;
}

void Plane::set( const double a, const double b, const double c, const double d )
{
    setA( a );
    setB( b );
    setC( c );
    setD( d );
}

void Plane::setA( const double value )
{
    m_a = value;
}

double Plane::a() const
{
    return m_a;
}

void Plane::setB( const double value )
{
    m_b = value;
}

double Plane::b() const
{
    return m_b;
}

void Plane::setC( const double value )
{
    m_c = value;
}

double Plane::c() const
{
    return m_c;
}

void Plane::setD( const double value )
{
    m_d = value;
}

double Plane::d() const
{
    return m_d;
}

double Plane::value( const double x, const double y, const double z )
{
    return m_a * x + m_b * y + m_c * z + m_d;
}

bool Plane::isValid() const
{
    return std::abs( m_a ) > DOUBLE_EPS || std::abs( m_b ) > DOUBLE_EPS || std::abs( m_c ) > DOUBLE_EPS || std::abs( m_d ) > DOUBLE_EPS;
}
