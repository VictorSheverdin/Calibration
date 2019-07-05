#pragma once

#include "src/common/supportwidgets.h"

class CalibrationChoiceDialog : public DialogBase
{
    Q_OBJECT

public:
    enum DocumentType { NONE, MONOCULAR_IMAGE, STEREO_IMAGE, MONOCULAR_CAMERA, STEREO_CAMERA };

    explicit CalibrationChoiceDialog( QWidget* parent = nullptr );

    DocumentType selectedType() const;

protected:
    QPointer< QListWidget > m_listWidget;

private:
    void initialize();

};

