#ifndef TREEWIDGET_H
#define TREEWIDGET_H

#include <QOpenGLWidget>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QPainter>
#include <QKeyEvent>
#include <QTimer>
#include <QOpenGLTexture>
#include "treemodeler.h"



class TreeWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT

public:
    explicit TreeWidget(QWidget *parent = 0);
    ~TreeWidget();


    enum PROJECT_MODE{_Perspective, _Ortho} m_projectMode;

    TreeModeler* m_tree;
    QTimer m_viewingTimer;

    void SetEyeDist(const QVector3D& c) { this->m_eyeDist = c;}
    void SetLeafTexture(const QString& filename);
    void SetBarkTexture(const QString& filename);

    bool m_isShowLeaves = true;
    bool m_polygonMode;
    bool m_isBarkTextured = true;
    bool m_isShowSeg;
    bool m_isShowMarkers=true;

protected:
    void initializeGL();
    void resizeGL(int w, int h);

    void paintGL();

    // shader programs
    QOpenGLShaderProgram* m_pointProgram;
    QOpenGLShaderProgram* m_skeletonProgram;
    QOpenGLShaderProgram* m_meshProgram;
    QOpenGLShaderProgram* m_leafProgram;
    // Textures
    QOpenGLTexture* m_barkTexture;
    QOpenGLTexture* m_leafTexture;

    void initShaders(QOpenGLShaderProgram *&m_program, const QString &shaderName, bool v, bool c, bool n, bool t);
    void setupShaders(QOpenGLShaderProgram*& m_program);
    void setupTexture(QOpenGLTexture*& texture,const QString& filename);

    // rendering related
    QMatrix4x4 m_projectMatrix;
    QMatrix4x4 m_viewMatrix;
    QMatrix4x4 m_modelMat;
    QVector3D m_eyePos,m_eyeDist;

    QPoint m_clickpos;
    double m_horAngle,m_verAngle;
    void mousePressEvent(QMouseEvent* event);
    void mouseMoveEvent(QMouseEvent* event);
    void mouseReleaseEvent(QMouseEvent* event);

    double distance;
    double scale;
    void wheelEvent(QWheelEvent* event);

    void keyPressEvent(QKeyEvent* e);

private slots:
    void OnViewingTimer();
};

#endif // TREEWIDGET_H
