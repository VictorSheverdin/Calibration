#include "src/common/precompiled.h"

#include "documentarea.h"

#include "supportwidgets.h"

DocumentArea::DocumentArea( QWidget *parent )
    : QMdiArea( parent )
{
    initialize();
}

void DocumentArea::initialize()
{
}

void DocumentArea::addWindow( QWidget *widget )
{
    auto subWindow = addSubWindow( widget );

    subWindow->showMaximized();

}

void DocumentArea::addDocument( DocumentBase *document )
{
    auto subWindow = addSubWindow( document );

    subWindow->showMaximized();

}

DocumentBase *DocumentArea::activeDocument() const
{
    return getDocument( activeSubWindow() );

}

DocumentBase *DocumentArea::currentDocument() const
{
    return getDocument( currentSubWindow() );
}

QList< DocumentBase * > DocumentArea::documentsList() const
{
    QList< DocumentBase * > ret;

    auto subWindowList = this->subWindowList();

    for ( auto &i : subWindowList ) {
        auto document = getDocument( i );

        if ( document )
            ret.push_back( document );

    }

    return ret;

}

DocumentBase *DocumentArea::getDocument( QMdiSubWindow *subWindow )
{
    if (subWindow)
        return dynamic_cast< DocumentBase * >( subWindow->widget() );

    return nullptr;

}
