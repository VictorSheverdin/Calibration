#include "src/common/precompiled.h"

#include "processorthread.h"

#include "src/common/functions.h"

#include "src/common/tictoc.h"

// ProcessorThread
ProcessorThread::ProcessorThread( QObject *parent )
    : QThread( parent )
{
    initialize();
}

void ProcessorThread::initialize()
{
}

bool ProcessorThread::process( const CvImage &img1 , const CvImage &img2 )
{
    bool res = false;

    if ( m_processMutex.tryLock() ) {
        if ( !isRunning() ) {
            m_img1 = img1;
            m_img2 = img2;
            start();
            res = true;
        }

        m_processMutex.unlock();
    }

    return res;

}

void ProcessorThread::setDetector( const std::shared_ptr<FeatureProcessorBase> &processor )
{
    m_detector = processor;
}

void ProcessorThread::setDescriptor( const std::shared_ptr<FeatureProcessorBase> &processor )
{
    m_descriptor = processor;
}

void ProcessorThread::setMatcher( std::shared_ptr< DescriptorMatcher > &matcher )
{
    m_matcher = matcher;
}

ProcessorResult ProcessorThread::result()
{
    ProcessorResult ret;

    m_resultMutex.lock();
    ret = m_result;
    m_resultMutex.unlock();

    return ret;
}

void ProcessorThread::run()
{
    TicToc timer;

    if ( !m_img1.empty() && !m_img2.empty() ) {

        std::vector< cv::KeyPoint > keyPoints1, keyPoints2;

        auto keypointProcessor = std::dynamic_pointer_cast< KeyPointProcessor >( m_detector );

        if ( keypointProcessor ) {

            keypointProcessor->extractKeypoints( m_img1, cv::Mat(), &keyPoints1 );
            keypointProcessor->extractKeypoints( m_img2, cv::Mat(), &keyPoints2 );

        }
        else {

            auto superglueProcessor = std::dynamic_pointer_cast< SuperGlueProcessor >( m_detector );

            if ( superglueProcessor )  {

                CvImage gray1, gray2;
                cv::cvtColor( m_img1, gray1, cv::COLOR_BGR2GRAY );
                cv::cvtColor( m_img2, gray2, cv::COLOR_BGR2GRAY );

                superglueProcessor->extractKeypoints( gray1, cv::Mat(), gray2, cv::Mat(), &keyPoints1, &keyPoints2 );
            }

        }

        CvImage result;

        auto flowProcessor = std::dynamic_pointer_cast< CPUFlowProcessor >( m_descriptor );

        if ( flowProcessor ) {

            std::vector< cv::Mat > img1Pyr, img2Pyr;
            flowProcessor->buildImagePyramid( m_img1, &img1Pyr );
            flowProcessor->buildImagePyramid( m_img2, &img2Pyr );

            std::vector< cv::Point2f > pts;

            for ( auto &i : keyPoints1 )
                pts.push_back( i.pt );

            std::vector< FlowTrackResult > trackResults;
            flowProcessor->track( img1Pyr, pts, img2Pyr, &trackResults );

            std::vector< cv::KeyPoint > procKeyPoints;

            std::vector< cv::DMatch > matches;

            for ( size_t i = 0; i < trackResults.size(); ++i ) {
                procKeyPoints.push_back( cv::KeyPoint( trackResults[i], 0. ) );
                matches.push_back( cv::DMatch( trackResults[i].index, i, 0 ) );
            }

            cv::drawMatches( m_img1, keyPoints1, m_img2, procKeyPoints, matches, result );

            m_resultMutex.lock();
            m_result.count = matches.size();
            m_result.procTime = timer.toc();
            m_result.result = result;
            m_resultMutex.unlock();

        }
        else {

            auto descriptorProcessor = std::dynamic_pointer_cast< DescriptorProcessor >( m_descriptor );

            if ( descriptorProcessor ) {

                cv::Mat descriptors1, descriptors2;

                descriptorProcessor->extractDescriptors( m_img1, keyPoints1, &descriptors1 );
                descriptorProcessor->extractDescriptors( m_img2, keyPoints2, &descriptors2 );

                std::vector< cv::DMatch > matches;

                if ( m_matcher ) {

                    m_matcher->match( keyPoints1, descriptors1, keyPoints2, descriptors2, &matches );

                    cv::drawMatches( m_img1, keyPoints1, m_img2, keyPoints2, matches, result );

                    m_resultMutex.lock();
                    m_result.count = matches.size();
                    m_result.procTime = timer.toc();
                    m_result.result = result;
                    m_resultMutex.unlock();

                }

            }
            else {

                auto superglueProcessor = std::dynamic_pointer_cast< SuperGlueProcessor >( m_descriptor );

                if ( superglueProcessor ) {

                    std::vector< cv::DMatch > matches;

                    superglueProcessor->match( keyPoints1, keyPoints2, &matches );

                    cv::drawMatches( m_img1, keyPoints1, m_img2, keyPoints2, matches, result );

                    m_resultMutex.lock();
                    m_result.count = matches.size();
                    m_result.procTime = timer.toc();
                    m_result.result = result;
                    m_resultMutex.unlock();


                }

            }

        }

        emit frameProcessed();

    }

}
