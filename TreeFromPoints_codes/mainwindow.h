#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "glwidget.h"
#include "treewidget.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    GLWidget* m_glWidget;
    TreeWidget* m_treeWidget;

    void initWidgets();
    void initLeafTextures();
    void initBarkTextures();

protected:
    void resizeEvent(QResizeEvent* e);
    void closeEvent(QCloseEvent* e);

protected slots:
    void slot_btnBackToOrigin();
    void slot_btnLoadModel();
    void slot_btnGetTrunk();
    void slot_btnConnectGraph();
    void slot_btnMinGraph();
    void slot_btnGetBins1();
    void slot_btnGetBins2();
    void slot_btnGetTreeSkeleton();
    void slot_btnOptimizeSkeleton();

    void slot_ckb_DisplaySkeletonWithColor();

    void slot_btnSaveDepthBuffer();

    void slot_cbxSetDisplayMode();
    void slot_spinChangeDisplayParameters();

    void slot_compareSkeletonWithOriginPts();
    void slot_startRoaming();

    void slot_updateRadius();

    void slot_cbxLeafTextures();
    void slot_cbxBarkTextures();

    void slot_leafAttributeChange();
    void slot_showLeaf();
    void slot_showBarkTexture();

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
