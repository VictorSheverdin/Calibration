#include "src/common/precompiled.h"

#include "optimizer.h"

#include "map.h"

#include "frame.h"
#include "mappoint.h"

#include <g2o/types/sba/types_six_dof_expmap.h>

namespace slam {

Optimizer::Optimizer()
{
    initialize();
}

void Optimizer::initialize()
{
}

void Optimizer::localAdjustment( slam::Map *map )
{
    adjustProcessed( map, 20 );
}

void Optimizer::adjustProcessed( slam::Map *map, const int count )
{
    if ( map ) {

        auto frames = map->frames();

        std::list< FramePtr > list;

        int counter = 0;

        for ( auto i = frames.rbegin(); i != frames.rend() && counter < count; ++i, ++counter )
            list.push_front( *i );

        adjust( list );

    }

}

void Optimizer::adjustStored( slam::Map *map, const int count )
{
    if ( map ) {

        auto frames = map->frames();

        std::list< FramePtr > list;

        int counter = 0;

        for ( auto i = frames.rbegin(); i != frames.rend() && counter < count; ++i, ++counter )
            if ( std::dynamic_pointer_cast< StereoFrame >( *i ) )
                list.push_front( *i );

        adjust( list );

    }
}

void Optimizer::adjust( std::list< FramePtr > &frames )
{
    g2o::SparseOptimizer optimizer;
    auto linearSolver = g2o::make_unique< g2o::LinearSolverEigen< g2o::BlockSolver_6_3::PoseMatrixType > >();
    auto solver = g2o::make_unique< g2o::OptimizationAlgorithmLevenberg >( g2o::make_unique< g2o::BlockSolver_6_3 >( std::move( linearSolver ) ) );
    optimizer.setAlgorithm( solver.get() );

    optimizer.setVerbose( false );

    std::map< MapPoint::PointPtr,  g2o::VertexSBAPointXYZ * > pointsMap;
    std::map< StereoFrameBase::FramePtr,  g2o::VertexSE3Expmap * > framesMap;

    int index = 0;

    for ( auto &i : frames ) {

        auto stereoFrame = std::dynamic_pointer_cast< StereoFrameBase >( i );

        if ( stereoFrame ) {

            auto leftFrame = stereoFrame->leftFrame();

            if ( leftFrame ) {

                auto frameVertex = new g2o::VertexSE3Expmap();
                frameVertex->setId( index++ );
                frameVertex->setFixed( false );
                frameVertex->setEstimate( leftFrame->se3Pose() );

                framesMap[ stereoFrame ] = frameVertex;

                optimizer.addVertex( frameVertex );

                auto framePoints = leftFrame->framePoints();

                for ( auto &j : framePoints ) {

                    if ( j ) {

                        auto mapPoint = j->mapPoint();

                        if ( mapPoint ) {

                            g2o::VertexSBAPointXYZ *pointVertex;

                            auto it = pointsMap.find( mapPoint );

                            if ( it == pointsMap.end() ) {

                                pointVertex = new g2o::VertexSBAPointXYZ();
                                pointVertex->setId( index++ );
                                pointVertex->setMarginalized( true );
                                pointVertex->setEstimate( mapPoint->eigenPoint() );

                                pointsMap[ mapPoint ] = pointVertex;

                                optimizer.addVertex( pointVertex );

                            }
                            else
                                pointVertex = it->second;

                            auto preojectEdge = new g2o::EdgeSE3ProjectXYZ();

                            preojectEdge->setVertex( 0, pointVertex );
                            preojectEdge->setVertex( 1, frameVertex );
                            preojectEdge->setMeasurement( j->eigenPoint() );
                            preojectEdge->setInformation( Eigen::Matrix2d::Identity() );

                            preojectEdge->fx = leftFrame->fx();
                            preojectEdge->fy = leftFrame->fy();
                            preojectEdge->cx = leftFrame->cx();
                            preojectEdge->cy = leftFrame->cy();

                            optimizer.addEdge( preojectEdge );

                            /*if ( j->stereoPoint() ) {

                                auto stereoEdge = new g2o::EdgeStereoSE3ProjectXYZ();

                                stereoEdge->setVertex( 0, pointsMap.at( j->mapPoint() ) );
                                stereoEdge->setVertex( 1, vertex );
                                stereoEdge->setMeasurement( j->eigenStereoPoint() );
                                stereoEdge->setInformation( Eigen::Matrix3d::Identity() );

                                stereoEdge->fx = leftFrame->fx();
                                stereoEdge->fy = leftFrame->fy();
                                stereoEdge->cx = leftFrame->cx();
                                stereoEdge->cy = leftFrame->cy();
                                stereoEdge->bf = j->bf();

                                optimizer.addEdge( stereoEdge );

                            }*/

                        }

                    }

                }

            }

        }

    }

    optimizer.initializeOptimization();
    optimizer.optimize( 5 );

    //Update

    for ( auto &i : pointsMap ) {

        if ( i.first && i.second ) {
            auto estimate = i.second->estimate();
            i.first->setEigenPoint( estimate );

        }

    }

    for ( auto &i : framesMap ) {

        if ( i.first && i.second ) {
            auto estimate = i.second->estimate();
            i.first->setLeftSe3Pose( estimate );

        }

    }

}

}
