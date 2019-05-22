template <class T>
T* MainWindowBase::getCurrentDocument() const
{
    auto currentDocument = m_documentArea->currentDocument();

    if ( currentDocument )
        return dynamic_cast< T* >( currentDocument );

    return nullptr;

}
