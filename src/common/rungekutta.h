#pragma once

#include <functional>
#include <Eigen/Geometry>

template < class T, size_t N >
class RungeKutta
{
public:
    using VectorNf = Eigen::Matrix< T, N, 1 >;

    RungeKutta( std::function< VectorNf( T, const VectorNf & ) > f );

    VectorNf integrate( T t, const VectorNf &in, T h );

private:
    std::function< VectorNf( T, const VectorNf & ) > _func;
};

#include "rungekutta.inl"
