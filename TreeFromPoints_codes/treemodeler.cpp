#include "treemodeler.h"
#include <QQueue>
#include <QDebug>
#include <QStack>
#define VERTEX_ATTRIBUTE  0
#define COLOUR_ATTRIBUTE  1
#define NORMAL_ATTRIBUTE  2
#define TEXTURE_ATTRIBUTE 3

int TreeSkelNode::maxLevel = 0;

TreeModeler::TreeModeler()
{

}

void TreeModeler::ConstructTreeStructure(const QVector<QPair<QVector3D, QVector3D> > &graph)
{
    qDebug()<<"Graph大小"<<graph.size();
    if(graph.size() == 0)
    {
        m_root = nullptr;
        return;
    }

    QVector<TreeSkelNode*> treeNodes;
    for(int i=0; i<graph.size(); ++i)
    {
        TreeSkelNode* node = new TreeSkelNode();
        node->a = graph[i].first;
        node->b = graph[i].second;

        treeNodes.append(node);
    }

    m_root = treeNodes[0];

    for(int i=1; i<treeNodes.size(); ++i)
    {
        TreeSkelNode* cur = treeNodes[i];
        TreeSkelNode* parent = nullptr;

        float min_dist = +999999;
        for(int k=0; k<i; ++k)
        {
            float dist = cur->a.distanceToPoint(treeNodes[k]->b);

            if(dist < min_dist)
            {
                min_dist = dist;
                parent = treeNodes[k];
            }
        }

        parent->childs.append(cur);
        cur->parent = parent;
    }
}

void TreeModeler::UpdateLevel()
{
    if(m_root == nullptr)
        return;

    TreeSkelNode::maxLevel = 0;
    m_root->level = 0;

    QQueue<TreeSkelNode*> queue;
    queue.enqueue(m_root);

    while(queue.size() != 0)
    {
        TreeSkelNode* cur = queue.dequeue();
        QVector3D curDir = (cur->b - cur->a).normalized();

        double optimial_angle = -999999;
        int best_id = -1;
        for(int i=0; i<cur->childs.size(); ++i)
        {
            TreeSkelNode* child = cur->childs[i];
            QVector3D childDir = (child->b - child->a).normalized();

            float angle = QVector3D::dotProduct(curDir, childDir);

            if(angle > optimial_angle) // 要寻找叉乘最大的。
            {
                optimial_angle = angle;
                best_id = i;
            }

            queue.enqueue(child);
        }

        for(int i=0; i<cur->childs.size(); ++i)
        {
            TreeSkelNode* child = cur->childs[i];
            if(i == best_id)
                child->level = cur->level;
            else
            {
                TreeSkelNode::maxLevel += 1;
                child->level = TreeSkelNode::maxLevel;
            }
        }
    }

    qDebug()<<"Level大小"<<TreeSkelNode::maxLevel;

}

void TreeModeler::UpdateBranchRadius()
{
    if(!this->m_root)
        return;

    QQueue<TreeSkelNode*> queue;
    QStack<TreeSkelNode*> parts;
    queue.enqueue(this->m_root);

    float radius_factor = m_branchRadiusFactor;

    while(!queue.isEmpty())
    {
        TreeSkelNode* cur = queue.dequeue();
        parts.push(cur);
        for(unsigned int i=0; i<cur->childs.size(); ++i)
            queue.enqueue(cur->childs[i]);
    }

    while(!parts.isEmpty())
    {
        TreeSkelNode* cur = parts.pop();

        if(cur->childs.size() == 0)
        {
            cur->rb = 0.45f;
            cur->ra = 0.45f;
        }
        else if (cur->childs.size() == 1)
        {
            cur->rb = cur->childs[0]->ra*radius_factor;
            cur->ra = cur->rb*radius_factor;
        }
        else
        {
            cur->rb = -9999999.0f;
            for(unsigned int i=0; i<cur->childs.size(); ++i)
            {
                if(cur->rb < cur->childs[i]->ra)
                    cur->rb = cur->childs[i]->ra;
            }
            cur->ra = cur->rb*radius_factor;
        }

        // Final refine into [0,2.0f]
        if(cur->rb >5.0f)
            cur->ra = cur->rb = 5.0f;
    }
}



void TreeModeler::BuildTreeMeshVBO()
{
    if(this->m_treeMeshVBO.vbo.isCreated())
        this->m_treeMeshVBO.vbo.release();
    if(this->m_leafMeshVBO.vbo.isCreated())
        this->m_leafMeshVBO.vbo.release();
    if(this->m_skeletonVBO.vbo.isCreated())
        this->m_skeletonVBO.vbo.release();

    if(!this->m_root)
        return;

    QVector<GLfloat> data;
    QVector<GLfloat> data_leaf;
    QVector<GLfloat> data_skeleton;

    QString _vt;
    QString _nt;
    QString _tt;
    QString _faces_barks;
    QString _faces_leafs;
    int vid = 0;
    int nid = 0;
    int tid = 0;

    // 首先构建以level为基准的levelList
    QQueue<TreeSkelNode*> queue;
    queue.enqueue(this->m_root);
    QVector<QVector<TreeSkelNode>> levelList;  // 根据level获取一串list
    levelList.fill(QVector<TreeSkelNode>(),TreeSkelNode::maxLevel+1);
    qDebug()<<"LevelList"<<levelList.size()<<TreeSkelNode::maxLevel;

    while(!queue.isEmpty())
    {
        auto cur = queue.dequeue();

        //qDebug()<<"  Cur Level:"<<cur->level;
        levelList[cur->level].push_back(*cur);

        for(unsigned int i=0; i<cur->childs.size(); ++i)
            queue.enqueue(cur->childs[i]);
    }

    //qDebug()<<"插值开始";
    // 基于不同的level进行插值
    for(unsigned int i=0; i<levelList.size(); ++i)
    {
        levelList[i] = this->Interpolate3D(levelList[i]);
    }

    //qDebug()<<"插值完毕";
    // 至此，parts顺序按照从根节点向上逐层存储着各级InterNodes
    //      parent则为对应的父亲结点
    for(unsigned int f=0; f<levelList.size(); ++f)
    {
        QVector<TreeSkelNode>& parts = levelList[f];

        for(int i=0; i<parts.size(); i++) // 每一个小骨节
        {
            QVector3D topPts[FACE_COUNT],botPts[FACE_COUNT],fNorms[FACE_COUNT];

            QVector3D dira = (parts[i].b-parts[i].a).normalized();  // 前端的平面法线 - bot
            QVector3D dirb = (parts[i].b-parts[i].a).normalized();  // 后端的平面法线 - top

            // 获得父亲部分的Top作为当前的bot
            if(i==0     // 如果是当前level的第一个节点
                    || QVector3D::dotProduct((parts[i-1].b-parts[i-1].a).normalized(),
                                             (parts[i].b-parts[i].a).normalized()) <0.8)  // 或者夹角比较小时
            {
                QVector3D norma = TreeMaths::getOneNormalVectorFromVector3D(dira);

                for(unsigned int k=0; k<FACE_COUNT; k++)
                {
                    QVector3D t_normA = QQuaternion::fromAxisAndAngle(dira,360.0f*k/(float)FACE_COUNT).rotatedVector(norma);
                    botPts[k] = parts[i].a + parts[i].ra*(t_normA).normalized();
                }
            }
            else
            {
                for(unsigned int m=0; m<FACE_COUNT; m++)
                    botPts[m] = parts[i-1].topPts[m];
            }

            // 之后创建TopBottom
            for(int k=0; k<FACE_COUNT; k++)
            {
                QVector3D pt = TreeMaths::RayToPlane(-dirb,parts[i].b,botPts[k],dira);  // 交点

                fNorms[k] = (pt-parts[i].b).normalized();

                topPts[k] = parts[i].b + parts[i].rb*fNorms[k];  // 得到topPts

                parts[i].topPts[k] = topPts[k];
            }

            for(int k=0; k<FACE_COUNT ; k++)  // 每一个侧面 facelet.
            {
                int id1 = k;
                int id2 = (k+1)%FACE_COUNT;

                data.push_back(botPts[id2].x());data.push_back(botPts[id2].y());data.push_back(botPts[id2].z());
                data.push_back(fNorms[id2].x());data.push_back(fNorms[id2].y());data.push_back(fNorms[id2].z());
                data.push_back((float)id2 / (float)FACE_COUNT);data.push_back(0.0f);data.push_back(1.0f);

                data.push_back(topPts[id2].x());data.push_back(topPts[id2].y());data.push_back(topPts[id2].z());
                data.push_back(fNorms[id2].x());data.push_back(fNorms[id2].y());data.push_back(fNorms[id2].z());
                data.push_back((float)id2 / (float)FACE_COUNT);data.push_back(1.0f);data.push_back(1.0f);

                data.push_back(topPts[id1].x());data.push_back(topPts[id1].y());data.push_back(topPts[id1].z());
                data.push_back(fNorms[id1].x());data.push_back(fNorms[id1].y());data.push_back(fNorms[id1].z());
                data.push_back((float)id1 / (float)FACE_COUNT);data.push_back(1.0f);data.push_back(1.0f);

                data.push_back(botPts[id1].x());data.push_back(botPts[id1].y());data.push_back(botPts[id1].z());
                data.push_back(fNorms[id1].x());data.push_back(fNorms[id1].y());data.push_back(fNorms[id1].z());
                data.push_back((float)id1 / (float)FACE_COUNT);data.push_back(0.0f);data.push_back(1.0f);

                vid += 4;
                tid += 4;
                nid += 4;

                _vt.push_back(QString("v %1 %2 %3\n").arg(botPts[id2].x()).arg(botPts[id2].y()).arg(botPts[id2].z()));
                _nt.push_back(QString("vn %1 %2 %3\n").arg(fNorms[id2].x()).arg(fNorms[id2].y()).arg(fNorms[id2].z()));
                _tt.push_back(QString("vt %1 %2 %3\n").arg((float)id2 / (float)FACE_COUNT).arg(0.0f).arg(1.0f));

                _vt.push_back(QString("v %1 %2 %3\n").arg(topPts[id2].x()).arg(topPts[id2].y()).arg(topPts[id2].z()));
                _nt.push_back(QString("vn %1 %2 %3\n").arg(fNorms[id2].x()).arg(fNorms[id2].y()).arg(fNorms[id2].z()));
                _tt.push_back(QString("vt %1 %2 %3\n").arg((float)id2 / (float)FACE_COUNT).arg(1.0f).arg(1.0f));

                _vt.push_back(QString("v %1 %2 %3\n").arg(topPts[id1].x()).arg(topPts[id1].y()).arg(topPts[id1].z()));
                _nt.push_back(QString("vn %1 %2 %3\n").arg(fNorms[id1].x()).arg(fNorms[id1].y()).arg(fNorms[id1].z()));
                _tt.push_back(QString("vt %1 %2 %3\n").arg((float)id1 / (float)FACE_COUNT).arg(1.0f).arg(1.0f));

                _vt.push_back(QString("v %1 %2 %3\n").arg(botPts[id1].x()).arg(botPts[id1].y()).arg(botPts[id1].z()));
                _nt.push_back(QString("vn %1 %2 %3\n").arg(fNorms[id1].x()).arg(fNorms[id1].y()).arg(fNorms[id1].z()));
                _tt.push_back(QString("vt %1 %2 %3\n").arg((float)id1 / (float)FACE_COUNT).arg(0.0f).arg(1.0f));

                _faces_barks.push_back(QString("f %1/%2/%3 %4/%5/%6 %7/%8/%9 %10/%11/%12\n")
                                       .arg(vid-3).arg(tid-3).arg(nid-3)
                                       .arg(vid-2).arg(tid-2).arg(nid-2)
                                       .arg(vid-1).arg(tid-1).arg(nid-1)
                                       .arg(vid-0).arg(tid-0).arg(nid-0));
            }

            // 生成skeleton data
            QVector3D rgb = QVector3D(1.0f, 0.0f, 0.0f);  // 该串list的skeleton颜色
            data_skeleton.push_back(parts[i].a.x());data_skeleton.push_back(parts[i].a.y());data_skeleton.push_back(parts[i].a.z());
            data_skeleton.push_back(rgb.x());data_skeleton.push_back(rgb.y());data_skeleton.push_back(rgb.z());
            data_skeleton.push_back(parts[i].b.x());data_skeleton.push_back(parts[i].b.y());data_skeleton.push_back(parts[i].b.z());
            data_skeleton.push_back(rgb.x());data_skeleton.push_back(rgb.y());data_skeleton.push_back(rgb.z());

            if((parts.size()-i)<m_leafRange)  // 生成leaves
            {
                for(unsigned int p=0; p<m_leafDensity; p++)
                {

                    QVector3D leafdir = TreeMaths::getOneNormalVectorFromVector3D(dirb);
                    leafdir = QQuaternion::fromAxisAndAngle(dirb,360.0f*TreeMaths::Random()).rotatedVector(leafdir);

//                    qDebug()<<TreeMaths::Random();
                    float ratio = TreeMaths::Random()/3.0f;
                    leafdir = ratio*leafdir+(1-ratio)*dirb;

                    leafdir.normalized();
                    QVector3D tan = QVector3D::crossProduct(dirb,leafdir).normalized();

                    float ratioPos = TreeMaths::Random();
                    QVector3D lfPos = ratioPos*parts[i].a + (1-ratioPos)*parts[i].b;

                    QVector3D _a = lfPos+ m_leafSize*tan;
                    QVector3D _b = lfPos - m_leafSize*tan;
                    QVector3D _c = _a + 2.0*m_leafSize*leafdir;
                    QVector3D _d = _b + 2.0*m_leafSize*leafdir;
                    QVector3D _n = QVector3D::crossProduct(_a-_b,_a-_c).normalized();
                    //qDebug()<<_a<<_b<<_c<<_d;
                    if(_n.y()<0)
                        _n = -_n;


                    data_leaf.push_back(_a.x());data_leaf.push_back(_a.y());data_leaf.push_back(_a.z());  //a
                    data_leaf.push_back(_n.x());data_leaf.push_back(_n.y());data_leaf.push_back(_n.z());
                    data_leaf.push_back(1.0f);data_leaf.push_back(1.0f);data_leaf.push_back(1.0f);
                    data_leaf.push_back(_b.x());data_leaf.push_back(_b.y());data_leaf.push_back(_b.z());  //b
                    data_leaf.push_back(_n.x());data_leaf.push_back(_n.y());data_leaf.push_back(_n.z());
                    data_leaf.push_back(1.0f);data_leaf.push_back(0.0f);data_leaf.push_back(1.0f);
                    data_leaf.push_back(_d.x());data_leaf.push_back(_d.y());data_leaf.push_back(_d.z());  //d
                    data_leaf.push_back(_n.x());data_leaf.push_back(_n.y());data_leaf.push_back(_n.z());
                    data_leaf.push_back(0.0f);data_leaf.push_back(0.0f);data_leaf.push_back(1.0f);
                    data_leaf.push_back(_c.x());data_leaf.push_back(_c.y());data_leaf.push_back(_c.z());  //d
                    data_leaf.push_back(_n.x());data_leaf.push_back(_n.y());data_leaf.push_back(_n.z());
                    data_leaf.push_back(0.0f);data_leaf.push_back(1.0f);data_leaf.push_back(1.0f);

                    vid += 4;
                    tid += 4;
                    nid += 4;

                    _vt.push_back(QString("v %1 %2 %3\n").arg(_a.x()).arg(_a.y()).arg(_a.z()));
                    _nt.push_back(QString("vn %1 %2 %3\n").arg(_n.x()).arg(_n.y()).arg(_n.z()));
                    _tt.push_back(QString("vt %1 %2 %3\n").arg(1.0f).arg(1.0f).arg(1.0f));

                    _vt.push_back(QString("v %1 %2 %3\n").arg(_b.x()).arg(_b.y()).arg(_b.z()));
                    _nt.push_back(QString("vn %1 %2 %3\n").arg(_n.x()).arg(_n.y()).arg(_n.z()));
                    _tt.push_back(QString("vt %1 %2 %3\n").arg(1.0f).arg(0.0f).arg(1.0f));

                    _vt.push_back(QString("v %1 %2 %3\n").arg(_d.x()).arg(_d.y()).arg(_d.z()));
                    _nt.push_back(QString("vn %1 %2 %3\n").arg(_n.x()).arg(_n.y()).arg(_n.z()));
                    _tt.push_back(QString("vt %1 %2 %3\n").arg(0.0f).arg(0.0f).arg(1.0f));

                    _vt.push_back(QString("v %1 %2 %3\n").arg(_c.x()).arg(_c.y()).arg(_c.z()));
                    _nt.push_back(QString("vn %1 %2 %3\n").arg(_n.x()).arg(_n.y()).arg(_n.z()));
                    _tt.push_back(QString("vt %1 %2 %3\n").arg(0.0f).arg(1.0f).arg(1.0f));

                    _faces_leafs.push_back(QString("f %1/%2/%3 %4/%5/%6 %7/%8/%9 %10/%11/%12\n")
                                           .arg(vid-3).arg(tid-3).arg(nid-3)
                                           .arg(vid-2).arg(tid-2).arg(nid-2)
                                           .arg(vid-1).arg(tid-1).arg(nid-1)
                                           .arg(vid-0).arg(tid-0).arg(nid-0));
                }
            }
        }
    }

    QFile file("./bark_texture.obj");
    file.open(QIODevice::WriteOnly);
    QTextStream ts(&file);

    ts<<_vt;
    ts<<_nt;
    ts<<_tt;
    ts<<"g bark\n"<<_faces_barks;
    ts<<"g leaf\n"<<_faces_leafs;
    file.close();

    m_treeMeshVBO.count = data.size()/9; // [!!!!!!] data: branch mesh vertices. (x,y,z, nx, ny, nz, texture_x, texture_y, 1.0)....
    // 除以9 = 顶点vertices的个数，每个vertex用了9个number.
    m_treeMeshVBO.vbo.create();
    m_treeMeshVBO.vbo.bind();
    m_treeMeshVBO.vbo.allocate(data.constData(),data.count()*sizeof(GLfloat));

    m_skeletonVBO.count = data_skeleton.size()/6;    // data_skeleton: tree graph nodes.

    m_skeletonVBO.vbo.create();
    m_skeletonVBO.vbo.bind();
    m_skeletonVBO.vbo.allocate(data_skeleton.constData(),data_skeleton.count()*sizeof(GLfloat));

    m_leafMeshVBO.count = data_leaf.size()/9;     //  [!!!!!!] data_leaf: leaf mesh vertices  (x,y,z, nx, ny, nz, texture_x, texture_y, 1.0)

    m_leafMeshVBO.vbo.create();
    m_leafMeshVBO.vbo.bind();
    m_leafMeshVBO.vbo.allocate(data_leaf.constData(),data_leaf.count()*sizeof(GLfloat));
    qDebug()<<data_leaf.size()<<m_leafMeshVBO.count;
}



//void TreeModeler::BuildTreeMeshVBO()
//{
//    if(this->m_treeMeshVBO.vbo.isCreated())
//        this->m_treeMeshVBO.vbo.release();
//    if(this->m_leafMeshVBO.vbo.isCreated())
//        this->m_leafMeshVBO.vbo.release();
//    if(this->m_skeletonVBO.vbo.isCreated())
//        this->m_skeletonVBO.vbo.release();

//    if(!this->m_root)
//        return;

//    QVector<GLfloat> data;
//    QVector<GLfloat> data_leaf;
//    QVector<GLfloat> data_skeleton;

//    QString _vt;
//    QString _nt;
//    QString _tt;
//    QString _faces_barks;
//    QString _faces_leafs;
//    int vid = 0;
//    int nid = 0;
//    int tid = 0;




//    // 首先构建以level为基准的levelList
//    QQueue<TreeSkelNode*> queue;
//    queue.enqueue(this->m_root);
//    QVector<QVector<TreeSkelNode>> levelList;  // 根据level获取一串list
//    levelList.fill(QVector<TreeSkelNode>(),TreeSkelNode::maxLevel+1);
//    qDebug()<<"LevelList"<<levelList.size()<<TreeSkelNode::maxLevel;

//    while(!queue.isEmpty())
//    {
//        auto cur = queue.dequeue();

//        //qDebug()<<"  Cur Level:"<<cur->level;
//        levelList[cur->level].push_back(*cur);

//        for(unsigned int i=0; i<cur->childs.size(); ++i)
//            queue.enqueue(cur->childs[i]);
//    }

//    //qDebug()<<"插值开始";
//    // 基于不同的level进行插值
////    for(unsigned int i=0; i<levelList.size(); ++i)
////    {
////        levelList[i] = this->Interpolate3D(levelList[i]);
////    }

//    //qDebug()<<"插值完毕";
//    // 至此，parts顺序按照从根节点向上逐层存储着各级InterNodes
//    //      parent则为对应的父亲结点
//    for(unsigned int f=0; f<levelList.size(); ++f)
//    {
//        QVector<TreeSkelNode>& parts = levelList[f];

//        for(int i=0; i<parts.size(); i++) // 每一个小骨节
//        {
//            QVector3D topPts[FACE_COUNT],botPts[FACE_COUNT],fNorms[FACE_COUNT];

//            QVector3D dira = (parts[i].b-parts[i].a).normalized();  // 前端的平面法线 - bot
//            QVector3D dirb = (parts[i].b-parts[i].a).normalized();  // 后端的平面法线 - top

//            // 获得父亲部分的Top作为当前的bot
//            if(i==0     // 如果是当前level的第一个节点
//                    || QVector3D::dotProduct((parts[i-1].b-parts[i-1].a).normalized(),
//                                             (parts[i].b-parts[i].a).normalized()) <0.8)  // 或者夹角比较小时
//            {
//                QVector3D norma = getOneNormalVectorFromVector3D(dira);

//                for(unsigned int k=0; k<FACE_COUNT; k++)
//                {
//                    QVector3D t_normA = QQuaternion::fromAxisAndAngle(dira,360.0f*k/(float)FACE_COUNT).rotatedVector(norma);
//                    botPts[k] = parts[i].a + parts[i].ra*(t_normA).normalized();
//                }
//            }
//            else
//            {
//                for(unsigned int m=0; m<FACE_COUNT; m++)
//                    botPts[m] = parts[i-1].topPts[m];
//            }

//            // 之后创建TopBottom
//            for(int k=0; k<FACE_COUNT; k++)
//            {
//                QVector3D pt = RayToPlane(-dirb,parts[i].b,botPts[k],dira);  // 交点

//                fNorms[k] = (pt-parts[i].b).normalized();

//                topPts[k] = parts[i].b + parts[i].rb*fNorms[k];  // 得到topPts

//                parts[i].topPts[k] = topPts[k];
//            }

//            for(int k=0; k<FACE_COUNT ; k++)  // 每一个侧面 facelet.
//            {
//                int id1 = k;
//                int id2 = (k+1)%FACE_COUNT;

//                data.push_back(botPts[id2].x());data.push_back(botPts[id2].y());data.push_back(botPts[id2].z());
//                data.push_back(fNorms[id2].x());data.push_back(fNorms[id2].y());data.push_back(fNorms[id2].z());
//                data.push_back((float)id2 / (float)FACE_COUNT);data.push_back(0.0f);data.push_back(1.0f);

//                data.push_back(topPts[id2].x());data.push_back(topPts[id2].y());data.push_back(topPts[id2].z());
//                data.push_back(fNorms[id2].x());data.push_back(fNorms[id2].y());data.push_back(fNorms[id2].z());
//                data.push_back((float)id2 / (float)FACE_COUNT);data.push_back(1.0f);data.push_back(1.0f);

//                data.push_back(topPts[id1].x());data.push_back(topPts[id1].y());data.push_back(topPts[id1].z());
//                data.push_back(fNorms[id1].x());data.push_back(fNorms[id1].y());data.push_back(fNorms[id1].z());
//                data.push_back((float)id1 / (float)FACE_COUNT);data.push_back(1.0f);data.push_back(1.0f);

//                data.push_back(botPts[id1].x());data.push_back(botPts[id1].y());data.push_back(botPts[id1].z());
//                data.push_back(fNorms[id1].x());data.push_back(fNorms[id1].y());data.push_back(fNorms[id1].z());
//                data.push_back((float)id1 / (float)FACE_COUNT);data.push_back(0.0f);data.push_back(1.0f);

//                vid += 4;
//                tid += 4;
//                nid += 4;

//                _vt.push_back(QString("v %1 %2 %3\n").arg(botPts[id2].x()).arg(botPts[id2].y()).arg(botPts[id2].z()));
//                _nt.push_back(QString("vn %1 %2 %3\n").arg(fNorms[id2].x()).arg(fNorms[id2].y()).arg(fNorms[id2].z()));
//                _tt.push_back(QString("vt %1 %2 %3\n").arg((float)id2 / (float)FACE_COUNT).arg(0.0f).arg(1.0f));

//                _vt.push_back(QString("v %1 %2 %3\n").arg(topPts[id2].x()).arg(topPts[id2].y()).arg(topPts[id2].z()));
//                _nt.push_back(QString("vn %1 %2 %3\n").arg(fNorms[id2].x()).arg(fNorms[id2].y()).arg(fNorms[id2].z()));
//                _tt.push_back(QString("vt %1 %2 %3\n").arg((float)id2 / (float)FACE_COUNT).arg(1.0f).arg(1.0f));

//                _vt.push_back(QString("v %1 %2 %3\n").arg(topPts[id1].x()).arg(topPts[id1].y()).arg(topPts[id1].z()));
//                _nt.push_back(QString("vn %1 %2 %3\n").arg(fNorms[id1].x()).arg(fNorms[id1].y()).arg(fNorms[id1].z()));
//                _tt.push_back(QString("vt %1 %2 %3\n").arg((float)id1 / (float)FACE_COUNT).arg(1.0f).arg(1.0f));

//                _vt.push_back(QString("v %1 %2 %3\n").arg(botPts[id1].x()).arg(botPts[id1].y()).arg(botPts[id1].z()));
//                _nt.push_back(QString("vn %1 %2 %3\n").arg(fNorms[id1].x()).arg(fNorms[id1].y()).arg(fNorms[id1].z()));
//                _tt.push_back(QString("vt %1 %2 %3\n").arg((float)id1 / (float)FACE_COUNT).arg(0.0f).arg(1.0f));

//                _faces_barks.push_back(QString("f %1/%2/%3 %4/%5/%6 %7/%8/%9 %10/%11/%12\n")
//                                       .arg(vid-3).arg(tid-3).arg(nid-3)
//                                       .arg(vid-2).arg(tid-2).arg(nid-2)
//                                       .arg(vid-1).arg(tid-1).arg(nid-1)
//                                       .arg(vid-0).arg(tid-0).arg(nid-0));
//            }

//            // 生成skeleton data
//            QVector3D rgb = QVector3D(1.0f, 0.0f, 0.0f);  // 该串list的skeleton颜色
//            data_skeleton.push_back(parts[i].a.x());data_skeleton.push_back(parts[i].a.y());data_skeleton.push_back(parts[i].a.z());
//            data_skeleton.push_back(rgb.x());data_skeleton.push_back(rgb.y());data_skeleton.push_back(rgb.z());
//            data_skeleton.push_back(parts[i].b.x());data_skeleton.push_back(parts[i].b.y());data_skeleton.push_back(parts[i].b.z());
//            data_skeleton.push_back(rgb.x());data_skeleton.push_back(rgb.y());data_skeleton.push_back(rgb.z());

//            if(i>=3 && (parts.size()-i)<10)  // 生成leaves
//            {
//                for(unsigned int p=0; p<8; p++)
//                {
//                    if(Random() > 0.5f)
//                        continue;
//                    QVector3D leafdir = getOneNormalVectorFromVector3D(dirb);
//                    leafdir = QQuaternion::fromAxisAndAngle(dirb,360.0f*Random()).rotatedVector(leafdir);


//                    float ratio = Random()/3.0f;
//                    leafdir = ratio*leafdir+(1-ratio)*dirb;

//                    leafdir += QVector3D(0,-1,0);
//                    leafdir.normalized();

//                    QVector3D tan = QVector3D::crossProduct(dirb,leafdir).normalized();

//                    QVector3D _a = parts[i].b + m_leafSize*tan;
//                    QVector3D _b = parts[i].b - m_leafSize*tan;
//                    QVector3D _c = _a + 2.0*m_leafSize*leafdir;
//                    QVector3D _d = _b + 2.0*m_leafSize*leafdir;
//                    QVector3D _n = QVector3D::crossProduct(_a-_b,_a-_c).normalized();
//                    //qDebug()<<_a<<_b<<_c<<_d;
//                    if(_n.y()<0)
//                        _n = -_n;


//                    data_leaf.push_back(_a.x());data_leaf.push_back(_a.y());data_leaf.push_back(_a.z());  //a
//                    data_leaf.push_back(_n.x());data_leaf.push_back(_n.y());data_leaf.push_back(_n.z());
//                    data_leaf.push_back(1.0f);data_leaf.push_back(1.0f);data_leaf.push_back(1.0f);
//                    data_leaf.push_back(_b.x());data_leaf.push_back(_b.y());data_leaf.push_back(_b.z());  //b
//                    data_leaf.push_back(_n.x());data_leaf.push_back(_n.y());data_leaf.push_back(_n.z());
//                    data_leaf.push_back(1.0f);data_leaf.push_back(0.0f);data_leaf.push_back(1.0f);
//                    data_leaf.push_back(_d.x());data_leaf.push_back(_d.y());data_leaf.push_back(_d.z());  //d
//                    data_leaf.push_back(_n.x());data_leaf.push_back(_n.y());data_leaf.push_back(_n.z());
//                    data_leaf.push_back(0.0f);data_leaf.push_back(0.0f);data_leaf.push_back(1.0f);
//                    data_leaf.push_back(_c.x());data_leaf.push_back(_c.y());data_leaf.push_back(_c.z());  //d
//                    data_leaf.push_back(_n.x());data_leaf.push_back(_n.y());data_leaf.push_back(_n.z());
//                    data_leaf.push_back(0.0f);data_leaf.push_back(1.0f);data_leaf.push_back(1.0f);

//                    vid += 4;
//                    tid += 4;
//                    nid += 4;

//                    _vt.push_back(QString("v %1 %2 %3\n").arg(_a.x()).arg(_a.y()).arg(_a.z()));
//                    _nt.push_back(QString("vn %1 %2 %3\n").arg(_n.x()).arg(_n.y()).arg(_n.z()));
//                    _tt.push_back(QString("vt %1 %2 %3\n").arg(1.0f).arg(1.0f).arg(1.0f));

//                    _vt.push_back(QString("v %1 %2 %3\n").arg(_b.x()).arg(_b.y()).arg(_b.z()));
//                    _nt.push_back(QString("vn %1 %2 %3\n").arg(_n.x()).arg(_n.y()).arg(_n.z()));
//                    _tt.push_back(QString("vt %1 %2 %3\n").arg(1.0f).arg(0.0f).arg(1.0f));

//                    _vt.push_back(QString("v %1 %2 %3\n").arg(_d.x()).arg(_d.y()).arg(_d.z()));
//                    _nt.push_back(QString("vn %1 %2 %3\n").arg(_n.x()).arg(_n.y()).arg(_n.z()));
//                    _tt.push_back(QString("vt %1 %2 %3\n").arg(0.0f).arg(0.0f).arg(1.0f));

//                    _vt.push_back(QString("v %1 %2 %3\n").arg(_c.x()).arg(_c.y()).arg(_c.z()));
//                    _nt.push_back(QString("vn %1 %2 %3\n").arg(_n.x()).arg(_n.y()).arg(_n.z()));
//                    _tt.push_back(QString("vt %1 %2 %3\n").arg(0.0f).arg(1.0f).arg(1.0f));

//                    _faces_leafs.push_back(QString("f %1/%2/%3 %4/%5/%6 %7/%8/%9 %10/%11/%12\n")
//                                           .arg(vid-3).arg(tid-3).arg(nid-3)
//                                           .arg(vid-2).arg(tid-2).arg(nid-2)
//                                           .arg(vid-1).arg(tid-1).arg(nid-1)
//                                           .arg(vid-0).arg(tid-0).arg(nid-0));
//                }
//            }
//        }
//    }

//    QFile file("./bark_texture.obj");
//    file.open(QIODevice::WriteOnly);
//    QTextStream ts(&file);

//    ts<<_vt;
//    ts<<_nt;
//    ts<<_tt;
//    ts<<"g bark\n"<<_faces_barks;
//    ts<<"g leaf\n"<<_faces_leafs;
//    file.close();

//    m_treeMeshVBO.count = data.size()/9; // [!!!!!!] data: branch mesh vertices. (x,y,z, nx, ny, nz, texture_x, texture_y, 1.0)....
//    // 除以9 = 顶点vertices的个数，每个vertex用了9个number.
//    m_treeMeshVBO.vbo.create();
//    m_treeMeshVBO.vbo.bind();
//    m_treeMeshVBO.vbo.allocate(data.constData(),data.count()*sizeof(GLfloat));

//    m_skeletonVBO.count = data_skeleton.size()/6;    // data_skeleton: tree graph nodes.

//    m_skeletonVBO.vbo.create();
//    m_skeletonVBO.vbo.bind();
//    m_skeletonVBO.vbo.allocate(data_skeleton.constData(),data_skeleton.count()*sizeof(GLfloat));

//    m_leafMeshVBO.count = data_leaf.size()/9;     //  [!!!!!!] data_leaf: leaf mesh vertices  (x,y,z, nx, ny, nz, texture_x, texture_y, 1.0)

//    m_leafMeshVBO.vbo.create();
//    m_leafMeshVBO.vbo.bind();
//    m_leafMeshVBO.vbo.allocate(data_leaf.constData(),data_leaf.count()*sizeof(GLfloat));
//    qDebug()<<data_leaf.size()<<m_leafMeshVBO.count;
//}



void TreeModeler::DrawTreeMeshVBO(QOpenGLShaderProgram *&program, const QMatrix4x4 &modelMat)
{
    if(this->m_treeMeshVBO.count <=0 ||!this->m_treeMeshVBO.vbo.isCreated())
        return;

    program->setUniformValue("mat_model",modelMat);

    m_treeMeshVBO.vbo.bind();

    program->enableAttributeArray(VERTEX_ATTRIBUTE);
    program->enableAttributeArray(NORMAL_ATTRIBUTE);
    program->enableAttributeArray(TEXTURE_ATTRIBUTE);

    program->setAttributeBuffer(VERTEX_ATTRIBUTE,   GL_FLOAT, 0,                3,9*sizeof(GLfloat));
    program->setAttributeBuffer(NORMAL_ATTRIBUTE,   GL_FLOAT, 3*sizeof(GLfloat),3,9*sizeof(GLfloat));
    program->setAttributeBuffer(TEXTURE_ATTRIBUTE,  GL_FLOAT, 6*sizeof(GLfloat),3,9*sizeof(GLfloat));

    glDrawArrays(GL_QUADS,0,this->m_treeMeshVBO.count);
}

void TreeModeler::DrawLeafMeshVBO(QOpenGLShaderProgram *&program, const QMatrix4x4 &modelMat)
{
    if(this->m_leafMeshVBO.count <=0 ||!this->m_leafMeshVBO.vbo.isCreated())
        return;

    program->setUniformValue("mat_model",modelMat);

    m_leafMeshVBO.vbo.bind();

    program->enableAttributeArray(VERTEX_ATTRIBUTE);
    program->enableAttributeArray(NORMAL_ATTRIBUTE);
    program->enableAttributeArray(TEXTURE_ATTRIBUTE);

    program->setAttributeBuffer(VERTEX_ATTRIBUTE,   GL_FLOAT, 0,                3,9*sizeof(GLfloat));
    program->setAttributeBuffer(NORMAL_ATTRIBUTE,   GL_FLOAT, 3*sizeof(GLfloat),3,9*sizeof(GLfloat));
    program->setAttributeBuffer(TEXTURE_ATTRIBUTE,  GL_FLOAT, 6*sizeof(GLfloat),3,9*sizeof(GLfloat));

    glDrawArrays(GL_QUADS,0,this->m_leafMeshVBO.count);
}


QVector<TreeSkelNode> TreeModeler::Interpolate3D(QVector<TreeSkelNode> list)
{

    if(list.size() <=4)
        return list;

    int size = list.size();
    double x[size];   // 0-(size-1)
    double _x[size]; double _y[size]; double _z[size];

    // 加入数据
    for(unsigned int k=0; k<list.size(); ++k)
    {
        x[k] = k;
        _x[k] = list[k].a.x();
        _y[k] = list[k].a.y();
        _z[k] = list[k].a.z();
    }

    SplineSpace::Spline cs_x(x,_x,size);
    SplineSpace::Spline cs_y(x,_y,size);
    SplineSpace::Spline cs_z(x,_z,size);

    int t = 3; // 中间新增的个数 t必须为奇数
    double  newX[size+(size-1)*t];   // 0 _ _ _ 1 _ _ _  2 _ _ _ 3 [size = 4] -> size=(size+(size-1)*t)
    double new_X[size+(size-1)*t];
    double new_Y[size+(size-1)*t];
    double new_Z[size+(size-1)*t];
    for (float i = 0.0f; i <= (size-1); i += 1.0f/(t+1))
        newX[(int)(i*(t+1))] = i;

    cs_x.MultiPointInterp(newX,size+(size-1)*t,new_X);
    cs_y.MultiPointInterp(newX,size+(size-1)*t,new_Y);
    cs_z.MultiPointInterp(newX,size+(size-1)*t,new_Z);

    QVector<TreeSkelNode> res;
    for (int i=0; i<size+(size-1)*t; ++i)
    {
        TreeSkelNode tmp = list[(int)newX[i]];
        tmp.a = QVector3D(new_X[i],new_Y[i],new_Z[i]);
        res.push_back(tmp);
    }

    for(unsigned int i=0; i<res.size()-1; ++i)
    {
        res[i].b = res[i+1].a;
    }
    return res;
}
