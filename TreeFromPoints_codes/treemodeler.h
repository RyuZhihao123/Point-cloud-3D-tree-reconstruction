#ifndef TREEMODELER_H
#define TREEMODELER_H
#include "treeutils.h"
#include "spline.h"
#include <QVector3D>
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <GL/gl.h>
#include <GL/glu.h>
#include <QFile>
#include <QTextStream>
#include <QOpenGLBuffer>
#include <QDebug>
#include <QVector>
#include <QTime>
#define FACE_COUNT 6
#define MAX_INT 999999
#define PI 3.14

struct GLBuffer
{
    QOpenGLBuffer vbo;
    int count;
};

class LeafNode
{
public:
    QVector3D pos;
    QVector3D dir1, dir2;
};

class TreeSkelNode
{
public:
    QVector3D a;
    QVector3D b;
    float ra = 0.3f;
    float rb = 0.3f;

    TreeSkelNode* parent = nullptr;
    QVector<TreeSkelNode*> childs;

    int level;  //  这个level用来描述不同的branch（从0 开始）。
    static int maxLevel;      // 最大的level数值
    QVector3D topPts[FACE_COUNT];
};

class TreeModeler
{
public:
    TreeModeler();

    TreeSkelNode* m_root = nullptr;


    float m_branchRadiusFactor = 1.05f;
    float m_leafSize = 5.0f;
    float m_leafDensity= 5.0f;
    float m_leafRange = 10.0f;

    void ConstructTreeStructure(const QVector< QPair<QVector3D,QVector3D> >& graph);  //
    void UpdateLevel();  // 第一步
    void UpdateBranchRadius();

    void BuildTreeMeshVBO();

    void DrawTreeMeshVBO(QOpenGLShaderProgram*& program, const QMatrix4x4& modelMat);

private:

//    QVector<InterNode> Interpolate3D(Vector<InterNode> list);

    // OpenGL Buffers
    GLBuffer m_skeletonVBO;
    GLBuffer m_treeMeshVBO;
    GLBuffer m_leafMeshVBO;
    GLBuffer m_pointVBO;

    QVector<TreeSkelNode> Interpolate3D(QVector<TreeSkelNode> list);

};

#endif // TREEMODELER_H
