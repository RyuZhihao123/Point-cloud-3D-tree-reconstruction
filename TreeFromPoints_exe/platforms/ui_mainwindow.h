#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QGroupBox *groupBox;
    QTabWidget *tabWidget;
    QWidget *tab;
    QPushButton *btnLoadMesh;
    QFrame *line;
    QPushButton *btnGetTrunk;
    QDoubleSpinBox *spinSearchRadius;
    QLabel *label;
    QFrame *line_2;
    QPushButton *btnConnectedGraph;
    QDoubleSpinBox *spinConnectInterval;
    QLabel *label_2;
    QPushButton *btnMinGraph;
    QPushButton *btnGetBins1;
    QPushButton *btnGetBins2;
    QPushButton *btnTreeSkeleton;
    QDoubleSpinBox *spinBinsRadius;
    QLabel *label_4;
    QDoubleSpinBox *spinBinsPtsCount;
    QLabel *label_5;
    QPushButton *btnBackToOrigin;
    QCheckBox *ckbCompareWithOriginPts;
    QPushButton *btnOptimizeSkeleton;
    QLabel *label_11;
    QLabel *label_12;
    QSpinBox *spinBox;
    QLabel *label_13;
    QSpinBox *spinBox_2;
    QCheckBox *ckbDisplaySkeletonWithColors;
    QWidget *widget;
    QComboBox *cbxDisplayMode;
    QLabel *label_3;
    QDoubleSpinBox *spinPointSize;
    QDoubleSpinBox *spinLineWidth;
    QLabel *label_6;
    QLabel *label_7;
    QSpinBox *spinCaptureCount;
    QPushButton *btnSaveDepthBuffer;
    QCheckBox *ckbRoaming;
    QLabel *label_8;
    QLabel *lblCurCaptureCount;
    QFrame *line_3;
    QLabel *label_10;
    QLabel *label_9;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(1400, 850);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        groupBox = new QGroupBox(centralWidget);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        groupBox->setGeometry(QRect(950, 0, 451, 791));
        QFont font;
        font.setFamily(QString::fromUtf8("\345\276\256\350\275\257\351\233\205\351\273\221"));
        groupBox->setFont(font);
        tabWidget = new QTabWidget(groupBox);
        tabWidget->setObjectName(QStringLiteral("tabWidget"));
        tabWidget->setGeometry(QRect(10, 40, 441, 751));
        tabWidget->setFont(font);
        tab = new QWidget();
        tab->setObjectName(QStringLiteral("tab"));
        btnLoadMesh = new QPushButton(tab);
        btnLoadMesh->setObjectName(QStringLiteral("btnLoadMesh"));
        btnLoadMesh->setGeometry(QRect(20, 30, 181, 51));
        btnLoadMesh->setFont(font);
        line = new QFrame(tab);
        line->setObjectName(QStringLiteral("line"));
        line->setGeometry(QRect(0, 100, 431, 16));
        line->setFont(font);
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);
        btnGetTrunk = new QPushButton(tab);
        btnGetTrunk->setObjectName(QStringLiteral("btnGetTrunk"));
        btnGetTrunk->setGeometry(QRect(20, 130, 151, 51));
        btnGetTrunk->setFont(font);
        spinSearchRadius = new QDoubleSpinBox(tab);
        spinSearchRadius->setObjectName(QStringLiteral("spinSearchRadius"));
        spinSearchRadius->setGeometry(QRect(260, 160, 121, 31));
        spinSearchRadius->setFont(font);
        spinSearchRadius->setDecimals(5);
        spinSearchRadius->setMinimum(0);
        spinSearchRadius->setMaximum(100.99);
        spinSearchRadius->setSingleStep(1e-5);
        spinSearchRadius->setValue(36.36);
        label = new QLabel(tab);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(230, 130, 171, 16));
        label->setFont(font);
        line_2 = new QFrame(tab);
        line_2->setObjectName(QStringLiteral("line_2"));
        line_2->setGeometry(QRect(190, 110, 16, 591));
        line_2->setFrameShape(QFrame::VLine);
        line_2->setFrameShadow(QFrame::Sunken);
        btnConnectedGraph = new QPushButton(tab);
        btnConnectedGraph->setObjectName(QStringLiteral("btnConnectedGraph"));
        btnConnectedGraph->setGeometry(QRect(20, 200, 151, 51));
        btnConnectedGraph->setFont(font);
        spinConnectInterval = new QDoubleSpinBox(tab);
        spinConnectInterval->setObjectName(QStringLiteral("spinConnectInterval"));
        spinConnectInterval->setGeometry(QRect(260, 240, 121, 31));
        spinConnectInterval->setFont(font);
        spinConnectInterval->setDecimals(5);
        spinConnectInterval->setMinimum(0);
        spinConnectInterval->setMaximum(100.99);
        spinConnectInterval->setSingleStep(1e-5);
        spinConnectInterval->setValue(36.36);
        label_2 = new QLabel(tab);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setGeometry(QRect(230, 200, 171, 16));
        label_2->setFont(font);
        btnMinGraph = new QPushButton(tab);
        btnMinGraph->setObjectName(QStringLiteral("btnMinGraph"));
        btnMinGraph->setGeometry(QRect(20, 270, 151, 51));
        btnMinGraph->setFont(font);
        btnGetBins1 = new QPushButton(tab);
        btnGetBins1->setObjectName(QStringLiteral("btnGetBins1"));
        btnGetBins1->setGeometry(QRect(20, 340, 151, 51));
        btnGetBins1->setFont(font);
        btnGetBins2 = new QPushButton(tab);
        btnGetBins2->setObjectName(QStringLiteral("btnGetBins2"));
        btnGetBins2->setGeometry(QRect(20, 410, 151, 51));
        btnGetBins2->setFont(font);
        btnTreeSkeleton = new QPushButton(tab);
        btnTreeSkeleton->setObjectName(QStringLiteral("btnTreeSkeleton"));
        btnTreeSkeleton->setGeometry(QRect(20, 480, 151, 51));
        QFont font1;
        font1.setFamily(QString::fromUtf8("\345\276\256\350\275\257\351\233\205\351\273\221"));
        font1.setBold(true);
        font1.setWeight(75);
        btnTreeSkeleton->setFont(font1);
        btnTreeSkeleton->setStyleSheet(QLatin1String("QPushButton\n"
"{\n"
"background-color: rgb(170, 24, 24);\n"
"}"));
        spinBinsRadius = new QDoubleSpinBox(tab);
        spinBinsRadius->setObjectName(QStringLiteral("spinBinsRadius"));
        spinBinsRadius->setGeometry(QRect(260, 360, 111, 31));
        spinBinsRadius->setFont(font);
        spinBinsRadius->setDecimals(5);
        spinBinsRadius->setMinimum(0);
        spinBinsRadius->setMaximum(200);
        spinBinsRadius->setSingleStep(1e-5);
        spinBinsRadius->setValue(10.36);
        label_4 = new QLabel(tab);
        label_4->setObjectName(QStringLiteral("label_4"));
        label_4->setGeometry(QRect(230, 300, 171, 51));
        label_4->setFont(font);
        label_4->setWordWrap(true);
        spinBinsPtsCount = new QDoubleSpinBox(tab);
        spinBinsPtsCount->setObjectName(QStringLiteral("spinBinsPtsCount"));
        spinBinsPtsCount->setGeometry(QRect(260, 460, 121, 31));
        spinBinsPtsCount->setFont(font);
        spinBinsPtsCount->setDecimals(5);
        spinBinsPtsCount->setMinimum(0);
        spinBinsPtsCount->setMaximum(200);
        spinBinsPtsCount->setSingleStep(1e-5);
        spinBinsPtsCount->setValue(6.36);
        label_5 = new QLabel(tab);
        label_5->setObjectName(QStringLiteral("label_5"));
        label_5->setGeometry(QRect(230, 400, 181, 51));
        label_5->setFont(font);
        label_5->setScaledContents(false);
        label_5->setWordWrap(true);
        btnBackToOrigin = new QPushButton(tab);
        btnBackToOrigin->setObjectName(QStringLiteral("btnBackToOrigin"));
        btnBackToOrigin->setGeometry(QRect(220, 30, 211, 51));
        btnBackToOrigin->setFont(font);
        ckbCompareWithOriginPts = new QCheckBox(tab);
        ckbCompareWithOriginPts->setObjectName(QStringLiteral("ckbCompareWithOriginPts"));
        ckbCompareWithOriginPts->setGeometry(QRect(230, 640, 161, 19));
        btnOptimizeSkeleton = new QPushButton(tab);
        btnOptimizeSkeleton->setObjectName(QStringLiteral("btnOptimizeSkeleton"));
        btnOptimizeSkeleton->setGeometry(QRect(20, 550, 151, 51));
        btnOptimizeSkeleton->setFont(font);
        label_11 = new QLabel(tab);
        label_11->setObjectName(QStringLiteral("label_11"));
        label_11->setGeometry(QRect(230, 500, 181, 51));
        label_11->setFont(font);
        label_11->setScaledContents(false);
        label_11->setWordWrap(true);
        label_12 = new QLabel(tab);
        label_12->setObjectName(QStringLiteral("label_12"));
        label_12->setGeometry(QRect(260, 540, 121, 51));
        label_12->setFont(font);
        label_12->setScaledContents(false);
        label_12->setWordWrap(true);
        spinBox = new QSpinBox(tab);
        spinBox->setObjectName(QStringLiteral("spinBox"));
        spinBox->setGeometry(QRect(380, 550, 51, 31));
        spinBox->setMinimum(1);
        spinBox->setValue(1);
        label_13 = new QLabel(tab);
        label_13->setObjectName(QStringLiteral("label_13"));
        label_13->setGeometry(QRect(260, 580, 121, 51));
        label_13->setFont(font);
        label_13->setScaledContents(false);
        label_13->setWordWrap(true);
        spinBox_2 = new QSpinBox(tab);
        spinBox_2->setObjectName(QStringLiteral("spinBox_2"));
        spinBox_2->setGeometry(QRect(380, 590, 51, 31));
        spinBox_2->setMinimum(1);
        spinBox_2->setValue(3);
        ckbDisplaySkeletonWithColors = new QCheckBox(tab);
        ckbDisplaySkeletonWithColors->setObjectName(QStringLiteral("ckbDisplaySkeletonWithColors"));
        ckbDisplaySkeletonWithColors->setGeometry(QRect(230, 680, 161, 19));
        tabWidget->addTab(tab, QString());
        widget = new QWidget();
        widget->setObjectName(QStringLiteral("widget"));
        cbxDisplayMode = new QComboBox(widget);
        cbxDisplayMode->setObjectName(QStringLiteral("cbxDisplayMode"));
        cbxDisplayMode->setGeometry(QRect(160, 30, 141, 31));
        label_3 = new QLabel(widget);
        label_3->setObjectName(QStringLiteral("label_3"));
        label_3->setGeometry(QRect(40, 40, 72, 15));
        spinPointSize = new QDoubleSpinBox(widget);
        spinPointSize->setObjectName(QStringLiteral("spinPointSize"));
        spinPointSize->setGeometry(QRect(130, 90, 121, 31));
        spinPointSize->setFont(font);
        spinPointSize->setDecimals(1);
        spinPointSize->setMinimum(1);
        spinPointSize->setMaximum(101);
        spinPointSize->setSingleStep(1);
        spinPointSize->setValue(5);
        spinLineWidth = new QDoubleSpinBox(widget);
        spinLineWidth->setObjectName(QStringLiteral("spinLineWidth"));
        spinLineWidth->setGeometry(QRect(130, 130, 121, 31));
        spinLineWidth->setFont(font);
        spinLineWidth->setDecimals(1);
        spinLineWidth->setMinimum(1);
        spinLineWidth->setMaximum(101);
        spinLineWidth->setSingleStep(1);
        spinLineWidth->setValue(1);
        label_6 = new QLabel(widget);
        label_6->setObjectName(QStringLiteral("label_6"));
        label_6->setGeometry(QRect(40, 100, 72, 15));
        label_7 = new QLabel(widget);
        label_7->setObjectName(QStringLiteral("label_7"));
        label_7->setGeometry(QRect(40, 140, 72, 15));
        spinCaptureCount = new QSpinBox(widget);
        spinCaptureCount->setObjectName(QStringLiteral("spinCaptureCount"));
        spinCaptureCount->setGeometry(QRect(170, 320, 91, 31));
        spinCaptureCount->setMaximum(1000);
        spinCaptureCount->setValue(1);
        btnSaveDepthBuffer = new QPushButton(widget);
        btnSaveDepthBuffer->setObjectName(QStringLiteral("btnSaveDepthBuffer"));
        btnSaveDepthBuffer->setGeometry(QRect(20, 310, 131, 51));
        btnSaveDepthBuffer->setFont(font);
        ckbRoaming = new QCheckBox(widget);
        ckbRoaming->setObjectName(QStringLiteral("ckbRoaming"));
        ckbRoaming->setGeometry(QRect(30, 240, 181, 51));
        label_8 = new QLabel(widget);
        label_8->setObjectName(QStringLiteral("label_8"));
        label_8->setGeometry(QRect(170, 370, 91, 31));
        lblCurCaptureCount = new QLabel(widget);
        lblCurCaptureCount->setObjectName(QStringLiteral("lblCurCaptureCount"));
        lblCurCaptureCount->setGeometry(QRect(270, 370, 151, 31));
        line_3 = new QFrame(widget);
        line_3->setObjectName(QStringLiteral("line_3"));
        line_3->setGeometry(QRect(0, 220, 431, 16));
        line_3->setFont(font);
        line_3->setFrameShape(QFrame::HLine);
        line_3->setFrameShadow(QFrame::Sunken);
        label_10 = new QLabel(widget);
        label_10->setObjectName(QStringLiteral("label_10"));
        label_10->setGeometry(QRect(270, 320, 91, 31));
        tabWidget->addTab(widget, QString());
        label_9 = new QLabel(groupBox);
        label_9->setObjectName(QStringLiteral("label_9"));
        label_9->setGeometry(QRect(11, 4, 441, 31));
        QFont font2;
        font2.setPointSize(14);
        label_9->setFont(font2);
        label_9->setAlignment(Qt::AlignCenter);
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1400, 17));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);

        tabWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", Q_NULLPTR));
        groupBox->setTitle(QString());
        btnLoadMesh->setText(QApplication::translate("MainWindow", "\345\212\240\350\275\275\347\202\271\344\272\221/\344\275\223\347\264\240", Q_NULLPTR));
        btnGetTrunk->setText(QApplication::translate("MainWindow", "\357\274\2101\357\274\211\347\241\256\345\256\232\344\270\273\345\271\262\345\214\272\345\237\237", Q_NULLPTR));
        label->setText(QApplication::translate("MainWindow", "1. \346\267\273\345\212\240\345\210\260\350\201\232\347\261\273\347\232\204\346\234\200\345\260\217\351\227\264\350\267\235\357\274\232", Q_NULLPTR));
        btnConnectedGraph->setText(QApplication::translate("MainWindow", "\357\274\2102\357\274\211\350\277\236\351\200\232\345\233\276", Q_NULLPTR));
        label_2->setText(QApplication::translate("MainWindow", "2. \350\277\236\346\216\245\347\232\204\346\234\200\345\244\247\351\227\264\350\267\235\357\274\232", Q_NULLPTR));
        btnMinGraph->setText(QApplication::translate("MainWindow", "\357\274\2103\357\274\211\346\234\200\345\260\217\347\224\237\346\210\220\346\240\221", Q_NULLPTR));
        btnGetBins1->setText(QApplication::translate("MainWindow", "\357\274\2104\357\274\211\345\210\222\345\210\206Bins", Q_NULLPTR));
        btnGetBins2->setText(QApplication::translate("MainWindow", "\357\274\2105\357\274\211\344\274\230\345\214\226Bins", Q_NULLPTR));
        btnTreeSkeleton->setText(QApplication::translate("MainWindow", "\357\274\2106\357\274\211\351\252\250\346\236\266", Q_NULLPTR));
        label_4->setText(QApplication::translate("MainWindow", "4.\345\275\222\344\270\272\344\270\200\344\270\252Bins\347\232\204\350\267\235\347\246\273\357\274\210\350\267\235\347\246\273\346\240\271\350\212\202\347\202\271\357\274\211", Q_NULLPTR));
        label_5->setText(QApplication::translate("MainWindow", "5.bins\347\273\206\345\214\226\346\227\266\357\274\214\347\202\271\346\234\200\345\260\217\351\227\264\350\267\235\347\232\204\346\224\276\345\244\247\345\200\215\346\225\260", Q_NULLPTR));
        btnBackToOrigin->setText(QApplication::translate("MainWindow", "\345\233\236\345\210\260\345\210\235\345\247\213\347\212\266\346\200\201", Q_NULLPTR));
        ckbCompareWithOriginPts->setText(QApplication::translate("MainWindow", "\344\270\216\345\216\237\345\247\213\347\202\271\344\272\221\346\257\224\350\276\203\357\274\237", Q_NULLPTR));
        btnOptimizeSkeleton->setText(QApplication::translate("MainWindow", "\357\274\2107\357\274\211\344\274\230\345\214\226", Q_NULLPTR));
        label_11->setText(QApplication::translate("MainWindow", "7.\351\252\250\346\236\266\344\274\230\345\214\226\347\273\206\350\212\202", Q_NULLPTR));
        label_12->setText(QApplication::translate("MainWindow", "\344\273\205\345\244\204\347\220\206\346\267\261\345\272\246\345\260\217\344\272\216", Q_NULLPTR));
        label_13->setText(QApplication::translate("MainWindow", "\347\210\266\345\255\220\346\267\261\345\272\246\345\267\256\345\244\247\344\272\216", Q_NULLPTR));
        ckbDisplaySkeletonWithColors->setText(QApplication::translate("MainWindow", "\346\230\276\347\244\272\351\252\250\346\236\266\346\267\261\345\272\246\344\277\241\346\201\257", Q_NULLPTR));
        tabWidget->setTabText(tabWidget->indexOf(tab), QApplication::translate("MainWindow", "\347\256\227\346\263\225\347\233\270\345\205\263", Q_NULLPTR));
        cbxDisplayMode->clear();
        cbxDisplayMode->insertItems(0, QStringList()
         << QApplication::translate("MainWindow", "\347\202\271\347\272\277", Q_NULLPTR)
         << QApplication::translate("MainWindow", "\347\202\271", Q_NULLPTR)
         << QApplication::translate("MainWindow", "\347\272\277", Q_NULLPTR)
        );
        label_3->setText(QApplication::translate("MainWindow", "\346\230\276\347\244\272\346\250\241\345\274\217\357\274\232", Q_NULLPTR));
        label_6->setText(QApplication::translate("MainWindow", "\347\202\271\345\244\247\345\260\217", Q_NULLPTR));
        label_7->setText(QApplication::translate("MainWindow", "\347\272\277\347\262\227\347\273\206", Q_NULLPTR));
        btnSaveDepthBuffer->setText(QApplication::translate("MainWindow", "\344\277\235\345\255\230\346\267\261\345\272\246\347\274\223\345\255\230...", Q_NULLPTR));
        ckbRoaming->setText(QApplication::translate("MainWindow", "\345\274\200\345\220\257\346\274\253\346\270\270\350\247\206\350\247\222", Q_NULLPTR));
        label_8->setText(QApplication::translate("MainWindow", "\345\275\223\345\211\215\344\277\235\345\255\230\357\274\232", Q_NULLPTR));
        lblCurCaptureCount->setText(QApplication::translate("MainWindow", "0 / 0", Q_NULLPTR));
        label_10->setText(QApplication::translate("MainWindow", "\345\271\205", Q_NULLPTR));
        tabWidget->setTabText(tabWidget->indexOf(widget), QApplication::translate("MainWindow", "\346\230\276\347\244\272\347\233\270\345\205\263", Q_NULLPTR));
        label_9->setText(QApplication::translate("MainWindow", "\345\210\230\345\277\227\346\265\251", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
