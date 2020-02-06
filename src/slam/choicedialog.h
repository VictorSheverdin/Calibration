#pragma once

#include "src/common/supportwidgets.h"

class ChoiceDialog : public DialogBase
{
    Q_OBJECT

public:
    enum DocumentType { NONE, IMAGES, CAMERA };

    explicit ChoiceDialog( QWidget* parent = nullptr );

    DocumentType selectedType() const;

protected:
    QPointer< QListWidget > m_listWidget;

private:
    void initialize();

};

