template < class T, size_t N >
RungeKutta< T, N >::RungeKutta( std::function< VectorNf( T, const VectorNf & ) > f )
    : _func( f )
{
}

template < class T, size_t N >
typename RungeKutta< T, N >::VectorNf RungeKutta< T, N >::integrate( T t, const VectorNf &in, T h )
{
    VectorNf k1, k2, k3, k4;
    k1 = _func( t, in );
    k2 = _func( t + h / 2, in + ( h / 2 ) * k1 );
    k3 = _func( t + h / 2, in + ( h / 2 ) * k2 );
    k4 = _func( t + h, in + h * k3 );

    return in + ( k1 + 2 * k2 + 2 * k3 + k4 ) / 6.;

}
