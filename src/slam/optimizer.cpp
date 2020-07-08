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

void Optimizer::adjust( std::list< FramePtr > &frames )
{    
    if ( frames.size() > 1 ) {

        g2o::SparseOptimizer optimizer;
        auto linearSolver = g2o::make_unique< g2o::LinearSolverEigen< g2o::BlockSolver_6_3::PoseMatrixType > >();
        auto solver = g2o::make_unique< g2o::OptimizationAlgorithmLevenberg >( g2o::make_unique< g2o::BlockSolver_6_3 >( std::move( linearSolver ) ) );
        optimizer.setAlgorithm( solver.get() );

        optimizer.setVerbose( false );

        std::map< MapPoint::ObjectPtr, std::shared_ptr< g2o::VertexSBAPointXYZ > > pointsMap;
        std::map< StereoFrameBase::FramePtr, std::shared_ptr< g2o::VertexSE3Expmap > > framesMap;

        std::list< std::shared_ptr< g2o::EdgeSE3ProjectXYZ > > projectEdges;
        std::list< std::shared_ptr< g2o::EdgeStereoSE3ProjectXYZ > > stereoEdges;

        int index = 0;

        bool fixed = true;

        for ( auto &i : frames ) {

            auto stereoFrame = std::dynamic_pointer_cast< StereoFrameBase >( i );

            if ( stereoFrame ) {

                auto leftFrame = stereoFrame->leftFrame();

                if ( leftFrame ) {

                    auto frameVertex = std::shared_ptr< g2o::VertexSE3Expmap >( new g2o::VertexSE3Expmap() );
                    frameVertex->setId( index++ );
                    frameVertex->setFixed( fixed );

                    if ( fixed )
                        fixed = false;

                    frameVertex->setEstimate( leftFrame->se3Pose() );

                    framesMap[ stereoFrame ] = frameVertex;

                    optimizer.addVertex( frameVertex.get() );

                    auto framePoints = leftFrame->framePoints();

                    for ( auto &j : framePoints ) {

                        if ( j ) {

                            auto mapPoint = j->mapPoint();

                            if ( mapPoint ) {

                                std::shared_ptr< g2o::VertexSBAPointXYZ > pointVertex;

                                auto it = pointsMap.find( mapPoint );

                                if ( it == pointsMap.end() ) {

                                    pointVertex = std::shared_ptr< g2o::VertexSBAPointXYZ >( new g2o::VertexSBAPointXYZ() );
                                    pointVertex->setId( index++ );
                                    pointVertex->setMarginalized( true );
                                    pointVertex->setEstimate( mapPoint->eigenPoint() );

                                    pointsMap[ mapPoint ] = pointVertex;

                                    optimizer.addVertex( pointVertex.get() );

                                }
                                else
                                    pointVertex = it->second;

                                auto projectEdge = std::shared_ptr< g2o::EdgeSE3ProjectXYZ >( new g2o::EdgeSE3ProjectXYZ() );
                                projectEdges.push_back( projectEdge );

                                projectEdge->setVertex( 0, pointVertex.get() );
                                projectEdge->setVertex( 1, frameVertex.get() );
                                projectEdge->setMeasurement( j->eigenPoint() );
                                projectEdge->setInformation( Eigen::Matrix2d::Identity() );

                                projectEdge->fx = leftFrame->fx();
                                projectEdge->fy = leftFrame->fy();
                                projectEdge->cx = leftFrame->cx();
                                projectEdge->cy = leftFrame->cy();

                                optimizer.addEdge( projectEdge.get() );

                                /*if ( j->stereoPoint() ) {

                                    auto stereoEdge = std::shared_ptr< g2o::EdgeStereoSE3ProjectXYZ >( new g2o::EdgeStereoSE3ProjectXYZ() );
                                    stereoEdges.push_back( stereoEdge );

                                    stereoEdge->setVertex( 0, pointVertex.get() );
                                    stereoEdge->setVertex( 1, frameVertex.get() );
                                    stereoEdge->setMeasurement( j->eigenStereoPoint() );
                                    stereoEdge->setInformation( Eigen::Matrix3d::Identity() );

                                    stereoEdge->fx = leftFrame->fx();
                                    stereoEdge->fy = leftFrame->fy();
                                    stereoEdge->cx = leftFrame->cx();
                                    stereoEdge->cy = leftFrame->cy();
                                    stereoEdge->bf = i->bf();

                                    optimizer.addEdge( stereoEdge.get() );

                                }*/

                            }

                        }

                    }

                }

            }

        }

        optimizer.initializeOptimization();
        optimizer.optimize( m_optimizationsCount );

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

}
