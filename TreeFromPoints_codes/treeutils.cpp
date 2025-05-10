#include "treeutils.h"

QVector3D TreeMaths::getOneNormalVectorFromVector3D(const QVector3D& _dir)
{
    if (_dir.x()== 0)
        return QVector3D(0, 0, -1);
    else
        return QVector3D(-_dir.z() / _dir.x(), 0, 1).normalized();
}

QVector3D TreeMaths::RayToPlane(const QVector3D& m_n,const QVector3D& m_a0,const QVector3D& p0,const QVector3D& u)
{
    float t = (QVector3D::dotProduct(m_n, m_a0) - QVector3D::dotProduct(m_n, p0)) / QVector3D::dotProduct(m_n, u);

    return p0 + t*u;
}

float TreeMaths::Random()
{
    return (qrand())/(float)RAND_MAX;
}
