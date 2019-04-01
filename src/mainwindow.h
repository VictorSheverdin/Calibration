#include <QMainWindow>

#include <QPointer>

class CalibrationWidgetBase;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow( const int cameraIndex, QWidget *parent = nullptr );
    explicit MainWindow( const int leftCameraIndex, const int rightCameraIndex, QWidget *parent = nullptr );

public slots:
    void grabFrame();
    void calculate();

    void settingsDialog();

protected:
    QPointer<CalibrationWidgetBase> m_widget;

    QPointer<QMenuBar> m_menuBar;
    QPointer<QStatusBar> m_statusBar;

    QPointer<QAction> m_newAction;
    QPointer<QAction> m_openAction;
    QPointer<QAction> m_saveAction;

    QPointer<QAction> m_exportAction;

    QPointer<QAction> m_grabAction;
    QPointer<QAction> m_autoGrabAction;
    QPointer<QAction> m_calculateAction;

    QPointer<QAction> m_settingsAction;

    QPointer<QAction> m_exitAction;

    QPointer<QAction> m_aboutAction;

    QPointer<QToolBar> m_toolBar;

    void setupActions();
    void setupMenus();
    void setupToolBars();
    void setupStatusBar();

private:
    void initialize( const int cameraIndex );
    void initialize( const int leftCameraIndex, const int rightCameraIndex );
    void initialize();

};
