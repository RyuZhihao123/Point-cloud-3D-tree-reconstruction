#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QOpenGLWidget>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QPainter>
#include <QKeyEvent>
#include <QTimer>
#include "tree.h"

class GLWidget : public QOpenGLWidget,public QOpenGLFunctions
{
    Q_OBJECT

public:

    enum DISPLAY_MODE
    {
        _LINE_POINT,
        _POINT,
        _LINE
    };

    explicit GLWidget(QWidget *parent = 0);
    ~GLWidget();

    Tree m_pointcloud;

    void loadModelDataFrom(const QString& filename);

    void setToOriginPoints();

    void getTrunk(double searchRadius = 4, bool showLeaf=false);  // 从输入点云 -> 仅保留枝干点云
    void connectGraph(double searchRadius = 4);
    void getMinGraph();
    void getBins1(double r);
    void getBins2(double minCount);
    void getTreeSkeleton();
    void optimizeSkeleton(int minLevel,int DepthDelta);

    void displaySkeletonDepthColor(bool b);   //

    void saveDepthBuffer(int amount, const QString &filename);

    void setDisplayMode(DISPLAY_MODE mode);

    void setDisplayParameters(float ptSize,float lnWidth);

    void compareSkeletonWithOriginPts(bool b);


    //void saveTreeStructures(const QString& filename);

    void startRoaming(bool b);

protected:
    void initializeGL();
    void resizeGL(int w, int h);

    void paintGL();

    bool m_isReadDepthBuffer;
    GLfloat* m_depthBuffer;

    QOpenGLShaderProgram* m_programPoints;
    QOpenGLShaderProgram* m_programGraph;
    QOpenGLShaderProgram* m_programFixedPipeLine; 
    QOpenGLShaderProgram* m_programLeafPoints;
    QList<QOpenGLShaderProgram*> m_programParts;

    QOpenGLShaderProgram* m_programTree;

    void initShaders(QOpenGLShaderProgram *&m_program, const QString &shaderName);

    QMatrix4x4 m_projectMatrix;
    QMatrix4x4 m_viewMatrix;
    QMatrix4x4 m_modelMat;

    void setupShaders(QOpenGLShaderProgram*& m_program);

    QPoint pos;
    double m_horAngle,m_verAngle;
    void mousePressEvent(QMouseEvent* event);
    void mouseMoveEvent(QMouseEvent* event);

    QVector3D m_eyePos,m_eyeDist;      //  define the view matrix;

    double distance;
    double scale;
    void wheelEvent(QWheelEvent* event);

    int m_mode;
    void keyPressEvent(QKeyEvent* e);

    void renderText(QString str,QPoint pos,QFont f,QColor c);

    void renderTree();   // 绘制最终的树

    // display Parameters
    DISPLAY_MODE m_displayMode;
    float m_pointSize;
    float m_lineWidth;

    QTimer m_timer;
    int m_captureAmount;
    QString m_captureDir;

protected slots:
    void slot_timerRoaming();
    void slot_timerSaveCaptures();

signals:
    void sig_CurCaptureCount(int,int);
};

#endif // GLWIDGET_H
