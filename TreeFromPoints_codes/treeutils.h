#ifndef TREEUTILS_H
#define TREEUTILS_H
#include <QVector3D>

class TreeMaths
{
public:
    static QVector3D getOneNormalVectorFromVector3D(const QVector3D& _dir);

    static QVector3D RayToPlane(const QVector3D& m_n,const QVector3D& m_a0,const QVector3D& p0,const QVector3D& u);

    static float Random();
};

#endif // TREEUTILS_H
