#pragma once

#include <QMdiArea>

#include "documentwidget.h"

class DocumentArea : public QMdiArea
{
    Q_OBJECT

public:
    explicit DocumentArea( QWidget *parent = nullptr );

    void addWindow( QWidget *widget );
    void addDocument( DocumentBase *document );

    DocumentBase *activeDocument() const;
    DocumentBase *currentDocument() const;

    QList< DocumentBase * > documentsList() const;

protected:
    static DocumentBase *getDocument( QMdiSubWindow *subWindow );

private:
    void initialize();

};
