#include "src/common/precompiled.h"

#include "writer.h"

Writer::Writer( const std::string &leftIp, const std::string &rightIp, QObject *parent )
    : QObject( parent ), m_camera( leftIp, rightIp )
{
    initialize();
}

void Writer::initialize()
{
    m_frameNumber = 0;

    connect( &m_camera, &StereoCamera::receivedFrame, this, &Writer::writeFrame );
}

void Writer::writeFrame()
{
    auto frame = m_camera.getFrame();

    if ( !frame.empty() ) {

        auto leftName = QString::number( m_frameNumber ) + "_left.png";
        auto rightName = QString::number( m_frameNumber ) + "_right.png";

        cv::imwrite( leftName.toStdString() , frame.leftFrame() );
        cv::imwrite( rightName.toStdString(), frame.rightFrame() );

        ++m_frameNumber;

        std::cout << "Frame " << m_frameNumber << " saved." << std::endl;

    }

}
