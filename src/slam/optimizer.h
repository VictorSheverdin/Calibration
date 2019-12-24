#pragma once

#include <opencv2/opencv.hpp>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>

namespace slam {

class Map;
class StereoFrameBase;

class Optimizer
{
public:
    using FramePtr = std::shared_ptr< StereoFrameBase >;

    Optimizer();

    void localAdjustment( slam::Map *map );

    void adjustProcessed( slam::Map *map, const int count );
    void adjustStored( slam::Map *map, const int count );

    void adjust( std::list< FramePtr > &frames );

private:
    void initialize();

};

}
