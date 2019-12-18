#pragma once

class Plane
{
public:
    Plane();
    Plane( const double a, const double b, const double c, const double d );

    void set( const double a, const double b, const double c, const double d );

    void setA( const double value );
    double a() const;

    void setB( const double value );
    double b() const;

    void setC( const double value );
    double c() const;

    void setD( const double value );
    double d() const;

    double value( const double x, const double y, const double z );

    bool isValid() const;

protected:
    double m_a;
    double m_b;
    double m_c;
    double m_d;

private:
    void initialize();
};
