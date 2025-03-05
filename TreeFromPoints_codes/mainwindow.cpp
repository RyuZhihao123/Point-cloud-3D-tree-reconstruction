#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QStyleFactory>
#include <QFileDialog>
#include "treewidget.h"

//#define LEAF_TEXTURES_PATH QString("E:/MyProjects/my_software/QtProjects/TreeFromPoints/Textures/Leaves/")
#define LEAF_TEXTURES_PATH QString("./Textures/Leaves/")

//#define BARK_TEXTURES_PATH QString("E:/MyProjects/my_software/QtProjects/TreeFromPoints/Textures/Barks/")
#define BARK_TEXTURES_PATH QString("./Textures/Barks/")

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    m_glWidget = new GLWidget(ui->centralWidget);
    m_treeWidget = new TreeWidget(ui->centralWidget);

    setWindowTitle("PtsToTree - 1RyuZhihao123(liuzhihao)");

    initWidgets();
    initLeafTextures();
    initBarkTextures();

    ui->mainToolBar->addWidget(ui->btnBackToOrigin);

    connect(ui->btnBackToOrigin,SIGNAL(clicked(bool)),this,SLOT(slot_btnBackToOrigin()));
    connect(ui->btnLoadMesh,SIGNAL(clicked(bool)),this,SLOT(slot_btnLoadModel()));
    connect(ui->btnGetTrunk,SIGNAL(clicked(bool)),this,SLOT(slot_btnGetTrunk()));
    connect(ui->btnConnectedGraph,SIGNAL(clicked(bool)),this,SLOT(slot_btnConnectGraph()));
    connect(ui->btnMinGraph,SIGNAL(clicked(bool)),this,SLOT(slot_btnMinGraph()));
    connect(ui->btnGetBins1,SIGNAL(clicked(bool)),this,SLOT(slot_btnGetBins1()));
    connect(ui->btnGetBins2,SIGNAL(clicked(bool)),this,SLOT(slot_btnGetBins2()));
    connect(ui->btnTreeSkeleton,SIGNAL(clicked(bool)),this,SLOT(slot_btnGetTreeSkeleton()));
    connect(ui->btnOptimizeSkeleton,SIGNAL(clicked(bool)),this,SLOT(slot_btnOptimizeSkeleton()));

    connect(ui->cbxDisplayMode,SIGNAL(currentIndexChanged(int)),this,SLOT(slot_cbxSetDisplayMode()));

    connect(ui->spinPointSize,SIGNAL(valueChanged(double)),this,SLOT(slot_spinChangeDisplayParameters()));
    connect(ui->spinLineWidth,SIGNAL(valueChanged(double)),this,SLOT(slot_spinChangeDisplayParameters()));

    connect(ui->ckbCompareWithOriginPts,SIGNAL(clicked(bool)),this,SLOT(slot_compareSkeletonWithOriginPts()));
    connect(ui->ckbDisplaySkeletonWithColors,SIGNAL(clicked(bool)),this,SLOT(slot_ckb_DisplaySkeletonWithColor()));

    connect(ui->spinBranchRadius, SIGNAL(valueChanged(double)), this, SLOT(slot_updateRadius()));
    connect(ui->cbxLeafTexture, SIGNAL(currentIndexChanged(int)), this, SLOT(slot_cbxLeafTextures()));
    connect(ui->cbxBarkTexture, SIGNAL(currentIndexChanged(int)), this, SLOT(slot_cbxBarkTextures()));
    connect(ui->spinLeafSize, SIGNAL(valueChanged(double)), this, SLOT(slot_leafAttributeChange()));
    connect(ui->spinLeafDensity, SIGNAL(valueChanged(double)), this, SLOT(slot_leafAttributeChange()));
    connect(ui->sliderLeafRange, SIGNAL(sliderReleased()), this, SLOT(slot_leafAttributeChange()));
    connect(ui->ckbShowTexturedBark, SIGNAL(clicked(bool)), this, SLOT(slot_showBarkTexture()));
    connect(ui->ckbHideLeaf, SIGNAL(clicked(bool)), this, SLOT(slot_showLeaf()));

//    // style
//    QFile file(":/qdarkstyle/style.qss");
//    if(file.open(QIODevice::ReadOnly))
//    {
//        QTextStream ts(&file);
//        QString strStyle = ts.readAll();
//        this->setStyleSheet(strStyle);
//        file.close();
//    }

    ui->lblPointCloud->raise();
    ui->lblTreeRecon->raise();
}

void MainWindow::slot_btnLoadModel()
{
    QString filename = QFileDialog::getOpenFileName(this,"Load Point Data",".","Point File (*.xyz)");

    if(filename == "")
        return;

    QDateTime time = QDateTime::currentDateTime();
    this->m_glWidget->loadModelDataFrom(filename);
    qDebug()<<"Time of Loading:"<<time.msecsTo(QDateTime::currentDateTime())<<"ms";
}

void MainWindow::resizeEvent(QResizeEvent *e)
{
    ui->groupBox->move(this->width()-ui->groupBox->width(),0);

    this->m_glWidget->setGeometry(3, ui->lblPointCloud->height()+ui->lblPointCloud->y(),
                                  (this->width() - ui->groupBox->width() - 10)/2.0f, this->height()-40);
    ui->lblTreeRecon->move(m_glWidget->x()+m_glWidget->width()+10, ui->lblTreeRecon->y());
    this->m_treeWidget->setGeometry(m_glWidget->x()+m_glWidget->width()+10,ui->lblTreeRecon->height()+ui->lblTreeRecon->y(),
                                    (this->width() - ui->groupBox->width() - 10)/2.0f, this->height()-40);

}

void MainWindow::initWidgets()
{
    ui->spinSearchRadius->setStyle(QStyleFactory::create("Macintosh"));
}

void MainWindow::slot_btnBackToOrigin()
{
    this->m_glWidget->setToOriginPoints();
}

void MainWindow::slot_btnGetTrunk()
{
    QDateTime time = QDateTime::currentDateTime();
    m_glWidget->getTrunk(ui->spinSearchRadius->value(), false);

    qDebug()<<"Time of finding trunk:"<<time.msecsTo(QDateTime::currentDateTime())<<"ms";
}

void MainWindow::slot_btnConnectGraph()
{
    QDateTime time = QDateTime::currentDateTime();
    m_glWidget->connectGraph(ui->spinConnectInterval->value());

}

void MainWindow::slot_btnGetBins1()
{

    m_glWidget->getBins1(this->ui->spinBinsRadius->value());

}

void MainWindow::slot_btnGetBins2()
{
    m_glWidget->getBins2(this->ui->spinBinsPtsCount->value());
}

void MainWindow::slot_btnMinGraph()
{
    QDateTime time = QDateTime::currentDateTime();
    m_glWidget->getMinGraph();

    qDebug()<<"Time of min value graph:"<<time.msecsTo(QDateTime::currentDateTime())<<"ms";
}

void MainWindow::slot_btnGetTreeSkeleton()
{
    QVector<QPair<QVector3D, QVector3D> > graph = m_glWidget->getTreeSkeleton();
    m_treeWidget->m_tree->ConstructTreeStructure(graph);
    m_treeWidget->m_tree->UpdateLevel();
    m_treeWidget->m_tree->UpdateBranchRadius();
    m_treeWidget->m_tree->BuildTreeMeshVBO();

    m_treeWidget->update();
}

void MainWindow::slot_btnOptimizeSkeleton()
{
    QVector<QPair<QVector3D, QVector3D> > graph = m_glWidget->optimizeSkeleton(1,3);

    m_treeWidget->m_tree->ConstructTreeStructure(graph);
    m_treeWidget->m_tree->UpdateLevel();
    m_treeWidget->m_tree->UpdateBranchRadius();
    m_treeWidget->m_tree->BuildTreeMeshVBO();

    m_treeWidget->update();
}

void MainWindow::slot_btnSaveDepthBuffer()
{
    QString filename = QFileDialog::getExistingDirectory(this, tr("保存深度截图"),
                                                         ".");

    if(filename == "")
        return;

    m_glWidget->saveDepthBuffer(1,filename);
}

void MainWindow::slot_cbxSetDisplayMode()
{
    m_glWidget->setDisplayMode((GLWidget::DISPLAY_MODE)this->ui->cbxDisplayMode->currentIndex());
}

void MainWindow::slot_spinChangeDisplayParameters()
{
    m_glWidget->setDisplayParameters(ui->spinPointSize->value(),
                                     ui->spinLineWidth->value());
}

void MainWindow::slot_compareSkeletonWithOriginPts()
{
    m_glWidget->compareSkeletonWithOriginPts(this->ui->ckbCompareWithOriginPts->isChecked());
}

void MainWindow::slot_startRoaming()
{
    m_glWidget->startRoaming(true);
}

void MainWindow::slot_ckb_DisplaySkeletonWithColor()
{
    m_glWidget->displaySkeletonDepthColor(ui->ckbDisplaySkeletonWithColors->isChecked());
}


void MainWindow::closeEvent(QCloseEvent *e)
{
    QStringList list;
    list<<PATH_BRANCH_PART<<PATH_ORIGIN_POINTS<<PATH_LEAVES_PART;

    for(unsigned int i=0; i<list.size(); i++)
    {
        QFile file(list[i]);

        if(file.exists())
        {
            file.remove();
        }
    }
    e->accept();
}


// ///////////////////////////////////////////
// 3D Tree Mesh显示相关：
// ///////////////////////////////////////////
void MainWindow::slot_updateRadius()
{
    m_treeWidget->m_tree->m_branchRadiusFactor = ui->spinBranchRadius->value();
    m_treeWidget->m_tree->UpdateBranchRadius();
    m_treeWidget->m_tree->BuildTreeMeshVBO();

    m_treeWidget->update();
}

void MainWindow::initLeafTextures()
{
    ui->cbxLeafTexture->clear();
    QDir dir(LEAF_TEXTURES_PATH);

    QFileInfoList list = dir.entryInfoList(QDir::Files);

    for(unsigned int i=0; i<list.size(); i++)
    {
        this->ui->cbxLeafTexture->addItem(QIcon(list[i].absoluteFilePath()),list[i].fileName());
    }
}

void MainWindow::initBarkTextures()
{
    ui->cbxBarkTexture->clear();
    QDir dir(BARK_TEXTURES_PATH);

    QFileInfoList list = dir.entryInfoList(QDir::Files);

    for(unsigned int i=0; i<list.size(); i++)
    {
        this->ui->cbxBarkTexture->addItem(QIcon(list[i].absoluteFilePath()),list[i].fileName());
    }
}


void MainWindow::slot_cbxLeafTextures()
{
    this->m_treeWidget->SetLeafTexture(LEAF_TEXTURES_PATH+this->ui->cbxLeafTexture->currentText());

    this->m_treeWidget->update();
}

void MainWindow::slot_cbxBarkTextures()
{
    this->m_treeWidget->SetBarkTexture(BARK_TEXTURES_PATH+this->ui->cbxBarkTexture->currentText());

    this->m_treeWidget->update();
}

void MainWindow::slot_leafAttributeChange()
{
    m_treeWidget->m_tree->m_leafSize = ui->spinLeafSize->value();
    m_treeWidget->m_tree->m_leafDensity = ui->spinLeafDensity->value();
    m_treeWidget->m_tree->m_leafRange = ui->sliderLeafRange->value();

    m_treeWidget->m_tree->BuildTreeMeshVBO();
    m_treeWidget->update();
}

void MainWindow::slot_showLeaf()
{
    m_treeWidget->m_isShowLeaves = ! ui->ckbHideLeaf->isChecked();
    m_treeWidget->update();
}

void MainWindow::slot_showBarkTexture()
{
    m_treeWidget->m_isBarkTextured = ui->ckbShowTexturedBark->isChecked();
    m_treeWidget->update();
}

MainWindow::~MainWindow()
{
    delete ui;
}
