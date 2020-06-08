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
