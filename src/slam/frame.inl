// ProcessedDenseFrame
template < class ProcessedFrameType >
ProcessedDenseFrame<ProcessedFrameType>::ProcessedDenseFrame( const MapPtr &parentMap )
    : ProcessedFrameType( parentMap )
{
}

template < class ProcessedFrameType >
typename ProcessedDenseFrame< ProcessedFrameType >::ObjectPtr ProcessedDenseFrame< ProcessedFrameType >::create( const MapPtr &parentMap )
{
    return ObjectPtr( new ProcessedDenseFrame< ProcessedFrameType >( parentMap ) );
}

template < class ProcessedFrameType >
void ProcessedDenseFrame< ProcessedFrameType >::processDenseCloud()
{
    auto leftFrame = this->leftFrame();
    auto rightFrame = this->rightFrame();

    if ( leftFrame && rightFrame ) {
        setPoints( this->parentMap()->parentWorld()->stereoProcessor().processPointList( leftFrame->image(), rightFrame->image() ) );

        // createOptimizationGrid();

    }

}

// DenseFrame
template < class DENSE_FRAME_TYPE >
void DenseFrame::replace( const std::shared_ptr< DENSE_FRAME_TYPE > &frame )
{
    StereoFrame::replace( frame );

    replaceProcedure( frame );

}

template < class DENSE_FRAME_TYPE >
void DenseFrame::replaceAndClean( const std::shared_ptr< DENSE_FRAME_TYPE > &frame )
{
    StereoFrame::replaceAndClean( frame );

    replaceProcedure( frame );

}

template < class DENSE_FRAME_TYPE >
void DenseFrame::replaceProcedure( const std::shared_ptr< DENSE_FRAME_TYPE > &frame )
{
    if ( frame ) {

        // TODO: Smarter selection

        for ( auto &i : frame->m_points )
            if ( cv::norm( i.point() ) < m_maximumLenght )
                m_points.push_back( i );

        setOptimizationGrid( frame->m_optimizationGrid );

    }

}
