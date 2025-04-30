#ifndef Tree_H
#define Tree_H
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
#include "params.h"
#include "kdtree.h"
#define MAX_INT 999999
#define PATH_ORIGIN_POINTS "./OriginPoints.pts"
#define PATH_BRANCH_PART "./BranchPart.pts"
#define PATH_LEAVES_PART "./LeavesPart.pts"

#define PI 3.14


using namespace nanoflann;

struct TreeNode
{
    QVector3D pos;   // 该结点的坐标
    double dist;  // 与父亲结点的距离

    QList<TreeNode*> childs;

    int level;

    // 对于其数组形态时，其父节点的id
    int parentId;

    TreeNode():dist(0),parentId(-1){}
    TreeNode(const QVector3D& r):pos(r),dist(0),parentId(-1){}
};

// 一个枝干单元
struct Bin
{
    QVector<QVector3D> pts;     // 这个Bin中的每个点
    QVector<int> ptsId;    // bin中的所有点，在原始数组中的id
    QList<Bin*> childs;

    int level;
    int depth;  // 从末尾往前数
    Bin* parentBin;

    QVector3D centerPos;
    Bin():parentBin(NULL),level(0),depth(0){}
};

struct BBox
{
    int* arr;  // x*y*z;
    float voxelSize;   // 一个体素的尺寸
    int rowCount;      // 每行拥有的体素个数

    void create(float size)   // 参数为一个体素的大小
    {
        this->voxelSize = size;
        this->rowCount = 200.0/size;

        this->arr = new int[rowCount*rowCount*rowCount];
        memset(this->arr,0,rowCount*rowCount*rowCount*sizeof(int));
    }

    void addPoints(const QVector<QVector3D>& vec)
    {
        for(unsigned int i=0; i<vec.size(); i++)
        {
            int x = (vec[i].x()+100)/this->voxelSize;
            int y = (vec[i].y()+100)/this->voxelSize;
            int z = (vec[i].z()+100)/this->voxelSize;
            x = x>=rowCount?rowCount-1:x;
            y = y>=rowCount?rowCount-1:y;
            z = z>=rowCount?rowCount-1:z;
            this->arr[(x*rowCount+y)*rowCount+z] ++;
        }
    }

    void getVertexArray(QVector<QVector3D>& vec)
    {
        vec.clear();
        for(int x=0; x<rowCount; x++)
        {
            for(int y=0; y<rowCount; y++)
            {
                for(int z=0; z<rowCount; z++)
                {
                    if(this->arr[(x*rowCount+y)*rowCount+z] >0)
                    {
                        vec.push_back(QVector3D(x*this->voxelSize-100.0+this->voxelSize/2.0,
                                                y*this->voxelSize-100.0+this->voxelSize/2.0,
                                                z*this->voxelSize-100.0+this->voxelSize/2.0));
                    }
                }
            }
        }
    }

    void clear()
    {
        delete [] this->arr;
    }
};

class Tree : public QOpenGLFunctions
{
public:
    Tree();

    enum PROCESS_STAGE
    {
        _STAGE_1,_STAGE_2,_STAGE_3,_STAGE_4,_STAGE_5,_STAGE_6,_STAGE_7
    } m_process_stage;

    // 辅助： 从点云中定位到根节点，并返回索引
    void setToOriginPoints();
    void clearAllBuffers();
    int locateRootindex(const QVector<QVector3D>& vec);
    void VectorToPointCloud(const QVector<QVector3D>& vec, PointCloud<double>& cloud);
    int getTreeCount(TreeNode*& root) const;
    void clearMinTree();
    void clearBinTree();
    void comperaSkeletonWithOriginPts(bool b);
    void displayFinalSkeletonWithColors(bool b);

    void enterCurrentStageMode(PROCESS_STAGE stage);
    // 点云  : 最初读入的三维点数据
    QVector<QVector3D> m_vertexs;

    // 分割后的枝干区域和树叶区域
    QVector<QVector3D> m_curVertexes;  // 当前正在分析的点云数据
    QVector<QVector3D> m_leafPart;
    QVector<QVector3D> m_skeletonPts;

    // 绘制用的连通图
    QVector<QVector<QPair<int,double> > > m_branchGraph;   // 对应在m_curVertexes中的索引号/距离

    // 骨架
    TreeNode* m_rootMinGraph;   // 最小生成树

    Bin* m_binRoot;     // bins
    QVector<Bin> bins;
    int m_maxBinLevel;

    // 体素化
    void voxelization(QVector<QVector3D>& vec);
    // [1] 从输入点云中获取枝干节点
    void getBranchAreaFromPoints(double searchRadius = 3.36);
    // [2] 生成初步连接图
    void generateConnectedGraph(double searchRadius = 3.36);
    // [3] Dijkstra算法 得到最小生成树
    void dijkstraMinGraph();

    // [5] 根据距离划分bins
    void divideIntoBins1(double r = 10);
    void divideIntoBins2(double minGapScale =0);

    // [6] 求解骨架
    QVector< QPair<QVector3D,QVector3D> > getFinalSkeleton();

    // [7] 优化
    QVector<QPair<QVector3D, QVector3D> > optimizeSkeleton();

    // 渲染有关
    // [1] 渲染点
    QOpenGLBuffer m_pointVBO;    // 渲染顶点的VBO
    int m_pointVBO_Vertex_Count;
    void createPointVBO(const QVector<QVector3D>& vecs, QVector<QVector3D> colors=QVector<QVector3D>());
    void drawPointVBO(QOpenGLShaderProgram*& program, const QMatrix4x4& modelMat);

    // [2] 渲染联通图
    QOpenGLBuffer m_graphVBO;
    int m_graphVBO_Vertex_Count;
    void createGraphVBO();  // 根据 m_branchGraph进行生成
    void drawGraphVBOWithOutColors(QOpenGLShaderProgram*& program, const QMatrix4x4& modelMat);

    // [3] 渲染对最终的骨架渲染有色彩的连通图
    bool m_isSkeletonBeColored;
    void createGraphVBOFromBin();
    void createGraphVBOFrom(const QVector<QPair<QVector3D,QVector3D>>& graph,
                            const QVector<QPair<int, int> > &graphDepth);

    QVector3D m_centerPos;  // 中心位置
    QVector3D m_rootPosition;         // 根节点索引

    // [3] 渲染bins 使用m_pointVBO
    void createPointVBOFromBins(QVector<Bin *> &bins);
    void createPointVBOFromBins(QVector<Bin> &bins);
    // 加载obj文件
    void readDataFromFile(QString filename);
    void loadPointsFrom(QVector<QVector3D>& vec,const QString& filename);
    void savePointsTo(QVector<QVector3D>& vec, const QString& filename);

    // Support Functions
    QVector3D getCenterFrom(QVector<QVector3D>& vertices);
};

#endif // Tree_H
