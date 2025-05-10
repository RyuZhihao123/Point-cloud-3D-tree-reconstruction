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


    // 首先构建以level为基准的levelList
    QQueue<TreeSkelNode*> queue;
    queue.enqueue(this->m_root);
    QVector<QVector<TreeSkelNode>> levelList;  // 根据level获取一串list
    levelList.fill(QVector<TreeSkelNode>(),TreeSkelNode::maxLevel+1);
    qDebug()<<"LevelList"<<levelList.size()<<TreeSkelNode::maxLevel;

    while(!queue.isEmpty())
    {
        auto cur = queue.dequeue();

        levelList[cur->level].push_back(*cur);

        for(unsigned int i=0; i<cur->childs.size(); ++i)
            queue.enqueue(cur->childs[i]);
    }

    for(unsigned int f=0; f<levelList.size(); ++f)
    {
        QVector<TreeSkelNode>& parts = levelList[f];

        for(int i=0; i<parts.size(); i++) // 每一个小骨节
        {
            QVector3D topPts[FACE_COUNT],botPts[FACE_COUNT],fNorms[FACE_COUNT];

            QVector3D dira = (parts[i].b-parts[i].a).normalized();
            QVector3D dirb = (parts[i].b-parts[i].a).normalized();


            if(i==0
                    || QVector3D::dotProduct((parts[i-1].b-parts[i-1].a).normalized(),
                                             (parts[i].b-parts[i].a).normalized()) <0.8)
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

            for(int k=0; k<FACE_COUNT; k++)
            {
                QVector3D pt = TreeMaths::RayToPlane(-dirb,parts[i].b,botPts[k],dira);

                fNorms[k] = (pt-parts[i].b).normalized();

                topPts[k] = parts[i].b + parts[i].rb*fNorms[k];

                parts[i].topPts[k] = topPts[k];
            }



            QVector3D rgb = QVector3D(1.0f, 0.0f, 0.0f);
            data_skeleton.push_back(parts[i].a.x());data_skeleton.push_back(parts[i].a.y());data_skeleton.push_back(parts[i].a.z());
            data_skeleton.push_back(rgb.x());data_skeleton.push_back(rgb.y());data_skeleton.push_back(rgb.z());
            data_skeleton.push_back(parts[i].b.x());data_skeleton.push_back(parts[i].b.y());data_skeleton.push_back(parts[i].b.z());
            data_skeleton.push_back(rgb.x());data_skeleton.push_back(rgb.y());data_skeleton.push_back(rgb.z());
        }
    }


    m_treeMeshVBO.vbo.allocate(data.constData(),data.count()*sizeof(GLfloat));
    m_skeletonVBO.count = data_skeleton.size()/6;    // data_skeleton: tree graph nodes.

    m_skeletonVBO.vbo.create();
    m_skeletonVBO.vbo.bind();
    m_skeletonVBO.vbo.allocate(data_skeleton.constData(),data_skeleton.count()*sizeof(GLfloat));
    qDebug()<<data_leaf.size()<<m_leafMeshVBO.count;
}



void TreeModeler::DrawTreeMeshVBO(QOpenGLShaderProgram *&program, const QMatrix4x4 &modelMat)
{
    program->setUniformValue("mat_model",modelMat);
    glDrawArrays(GL_QUADS,0,this->m_treeMeshVBO.count);
}




