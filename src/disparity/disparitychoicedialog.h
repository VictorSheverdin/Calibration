#pragma once

#include "src/common/supportwidgets.h"

class DisparityChoiceDialog : public DialogBase
{
    Q_OBJECT

public:
    enum DocumentType { NONE, IMAGES, CAMERA };

    explicit DisparityChoiceDialog( QWidget* parent = nullptr );

    DocumentType selectedType() const;

protected:
    QPointer< QListWidget > m_listWidget;

private:
    void initialize();

};

