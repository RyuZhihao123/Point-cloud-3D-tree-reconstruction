#include "glwidget.h"

QVector3D colors[7]= {
    QVector3D(0,0.811765,0.811765),QVector3D(0.811765,0,0.811765),QVector3D(0.886275,0.490196,0.06274511),
    QVector3D(0,0.25098,0.811765), QVector3D(0.811765,0.811765,0),QVector3D(0.25098,0.811765,0),
    QVector3D(1,1,1)
};

GLWidget::GLWidget(QWidget *parent) :
    QOpenGLWidget(parent)
{
    m_horAngle = m_verAngle = 45.0;
    distance = 300.0;
    scale = 1.0;

    m_eyeDist = QVector3D(0,0,0);

    this->setFocusPolicy(Qt::StrongFocus);

    m_mode = 0;

    m_depthBuffer = NULL;

    m_isReadDepthBuffer = false;

    this->m_displayMode = DISPLAY_MODE::_LINE_POINT;
    this->m_pointSize = 5;
    this->m_lineWidth = 1;
}

void GLWidget::initShaders(QOpenGLShaderProgram*& m_program,const QString& shaderName)
{

#define VERTEX_ATTRIBUTE_BRANCH 0
#define COLOR_ATTRIBUTE_BRANCH 1
    m_program = new QOpenGLShaderProgram(this);

    m_program->addShaderFromSourceFile(QOpenGLShader::Vertex,  QString(":/ss/res/shaders/%1VShader.glsl").arg(shaderName));
    m_program->addShaderFromSourceFile(QOpenGLShader::Fragment,QString(":/ss/res/shaders/%1FShader.glsl").arg(shaderName));


    m_program->bindAttributeLocation("a_position",VERTEX_ATTRIBUTE_BRANCH);
    m_program->bindAttributeLocation("a_color"   ,COLOR_ATTRIBUTE_BRANCH);

    m_program->link();
    m_program->bind();
}

void GLWidget::initializeGL()
{
    initializeOpenGLFunctions();

    glClearColor(1.0,1.0,1.0,1.0);

    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glShadeModel(GL_SMOOTH);
    glEnable(GL_POINT_SMOOTH);
    glEnable(GL_BLEND);
    glEnable(GL_ALPHA_TEST);

    initShaders(m_programPoints,"Points");
    initShaders(m_programGraph,"Graph");
    initShaders(m_programFixedPipeLine,"Single");

    //this->m_pointcloud.readDataFromFile("C:/Users/LiewDennis/Desktop/Tree.obj");

    m_eyeDist = m_pointcloud.m_centerPos;

    this->m_pointcloud.createPointVBO(this->m_pointcloud.m_vertexs);

    qDebug()<<"init结束";
}

void GLWidget::resizeGL(int w, int h)
{
    glViewport(0,0,width(),height());

    m_projectMatrix.setToIdentity();
    m_projectMatrix.perspective(45.0f,(float)w/(float)h,0.1f,1000.0f);
}

void GLWidget::paintGL()
{
    // 绘制一个坐标系
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    m_eyePos = m_eyeDist+QVector3D(distance*cos(PI*m_verAngle/180.0)*cos(PI*m_horAngle/180.0),
                                   distance*sin(PI*m_verAngle/180.0)+2,
                                   distance*cos(PI*m_verAngle/180.0)*sin(PI*m_horAngle/180.0));
    m_viewMatrix.setToIdentity();
    m_viewMatrix.lookAt(m_eyePos,m_eyeDist,QVector3D(0,1,0));

    m_modelMat.setToIdentity();
    m_modelMat.scale(QVector3D(scale,scale,scale));

    // 绘制最初的点云
    if(this->m_displayMode == _LINE_POINT || this->m_displayMode == _POINT)
    {
        glPointSize(this->m_pointSize);
        setupShaders(m_programPoints);
        m_pointcloud.drawPointVBO(this->m_programPoints,this->m_modelMat);
    }
    if(this->m_displayMode == _LINE_POINT || this->m_displayMode == _LINE)
    {
        glLineWidth(this->m_lineWidth);
        setupShaders(m_programGraph);
        m_pointcloud.drawGraphVBOWithOutColors(this->m_programGraph,this->m_modelMat);
    }

    setupShaders(m_programFixedPipeLine);
    m_programFixedPipeLine->setUniformValue("mat_model",this->m_modelMat);

//    glPointSize(13);
//    glColor3f(1,0,0);
//    glBegin(GL_POINTS);
//        glVertex3f(this->m_pointcloud.m_rootPosition.x(),this->m_pointcloud.m_rootPosition.y()
//                   ,this->m_pointcloud.m_rootPosition.z());
//    glEnd();

//    setupShaders(m_programTree);
//    renderTree();
//    renderText("Pts_To_Tree 刘志浩",QPoint(15,30),QFont("Times New Roman",11),QColor(255,255,255));
//    renderText(QString("PtAmount: %1").arg(m_pointcloud.m_vertexs.size()),QPoint(15,50),QFont("Times New Roman",11),QColor(255,255,255));

//    glBegin(GL_LINES);
//        glVertex3f(-100.0f, -100.0f, -100.0f);
//        glVertex3f(-100.0f, 100.0f, -100.0f);
//        glVertex3f(-100.0f, -100.0f, -100.0f);
//        glVertex3f(-100.0f, -100.0f, 100.0f);
//        glVertex3f(-100.0f, -100.0f, -100.0f);
//        glVertex3f(100.0f, -100.0f, -100.0f);
//    glEnd();

   // if(this->m_isReadDepthBuffer)
    {
        if(m_depthBuffer)
            delete[] m_depthBuffer;
        m_depthBuffer = new GLfloat[width()*height()];
        glReadPixels(0,0,width(),height(),GL_DEPTH_COMPONENT,GL_FLOAT,m_depthBuffer);
        this->m_isReadDepthBuffer = false;
    }

    glFlush();
}

void GLWidget::setupShaders(QOpenGLShaderProgram *&m_program)
{
    m_program->bind();

    m_program->setUniformValue("mat_projection",m_projectMatrix);
    m_program->setUniformValue("mat_view",m_viewMatrix);
}

void GLWidget::renderText(QString str,QPoint pos,QFont f,QColor c)
{
    glDisable(GL_DEPTH_TEST);
    QPainter painter;
    painter.begin(this);
    QPen pen;
    pen.setColor(c);
    painter.setFont(f);
    painter.setPen(pen);

    painter.drawText(pos,str);

    painter.end();
    glEnable(GL_DEPTH_TEST);
}

void GLWidget::setToOriginPoints()
{
    this->m_pointcloud.setToOriginPoints();
    this->update();
}

void GLWidget::loadModelDataFrom(const QString &filename)
{
    this->m_pointcloud.readDataFromFile(filename);

    this->m_eyeDist = this->m_pointcloud.m_centerPos;
    this->update();
}

void GLWidget::getTrunk(double searchRadius,bool showLeaf)
{
    this->m_pointcloud.getBranchAreaFromPoints(searchRadius);

    if(!showLeaf)
    {
        this->m_pointcloud.createPointVBO(this->m_pointcloud.m_curVertexes);
    }
    else
    {
        QVector<QVector3D> colors;
        QVector<QVector3D> vecs = this->m_pointcloud.m_curVertexes+this->m_pointcloud.m_leafPart;
        colors.resize(vecs.size());

        QVector3D white(1,1,1);
        QVector3D   red(1,0,0);
        for(unsigned int i=0; i<vecs.size(); i++)
        {
            if(i<this->m_pointcloud.m_curVertexes.size())
                colors[i] = white;
            if(i>=this->m_pointcloud.m_curVertexes.size())
                colors[i] = red;
        }

        this->m_pointcloud.createPointVBO(vecs,colors);

        vecs.clear();
        colors.clear();
    }
    this->update();
}

void GLWidget::connectGraph(double searchRadius)
{
    this->m_pointcloud.generateConnectedGraph(searchRadius);
    this->m_pointcloud.createGraphVBO();
    this->update();
}

void GLWidget::getMinGraph()
{
    this->m_pointcloud.dijkstraMinGraph();
    this->update();
}

void GLWidget::getBins1(double r)
{
    this->m_pointcloud.divideIntoBins1(r);
    this->update();

}

void GLWidget::getBins2(double minCount)
{
    this->m_pointcloud.divideIntoBins2(minCount);
    this->update();
}

QVector<QPair<QVector3D, QVector3D> >  GLWidget::getTreeSkeleton()
{
    auto graph = this->m_pointcloud.getFinalSkeleton();
    this->update();

    return graph;
}

QVector<QPair<QVector3D, QVector3D> > GLWidget::optimizeSkeleton(int minLevel, int DepthDelta)
{
    auto graph = this->m_pointcloud.optimizeSkeleton();
    this->update();

    return graph;
}

void GLWidget::displaySkeletonDepthColor(bool b)
{
    this->m_pointcloud.displayFinalSkeletonWithColors(b);
    this->update();
}

void GLWidget::saveDepthBuffer(int amount,const QString& filename)
{
    m_timer.stop();
    disconnect(&m_timer,SIGNAL(timeout()),0,0);
    connect(&m_timer,SIGNAL(timeout()),this,SLOT(slot_timerSaveCaptures()));
    m_timer.start(30);

    this->m_captureAmount = amount;
    this->m_captureDir = filename;
}

void GLWidget::startRoaming(bool b)
{
    m_timer.stop();
    disconnect(&m_timer,SIGNAL(timeout()),0,0);
    connect(&m_timer,SIGNAL(timeout()),this,SLOT(slot_timerRoaming()));

    if(b)
        m_timer.start(20);
    else
        m_timer.stop();
}

void GLWidget::setDisplayMode(DISPLAY_MODE mode)
{
    this->m_displayMode = mode;
    this->update();
}

void GLWidget::setDisplayParameters(float ptSize, float lnWidth)
{
    this->m_pointSize = ptSize;
    this->m_lineWidth = lnWidth;
    this->update();
}

void GLWidget::compareSkeletonWithOriginPts(bool b)
{
    this->m_pointcloud.comperaSkeletonWithOriginPts(b);
    this->update();
}

void GLWidget::mousePressEvent(QMouseEvent *event)
{
    pos = event->pos();
}

void GLWidget::mouseMoveEvent(QMouseEvent *event)
{
    // If control and left button;
    if(event->buttons() == Qt::LeftButton)
    {
        QPoint cur = event->pos();

        double dx = (cur.x() - pos.x())/5.0;
        double dy = (cur.y() - pos.y())/5.0;

        m_horAngle +=dx;
        if(m_verAngle+dy <90.0 && m_verAngle+dy>-90.0)
            m_verAngle +=dy;

        pos = cur;
        update();
    }
}

void GLWidget::keyPressEvent(QKeyEvent *e)
{
    if(e->key() == Qt::Key_C)
    {
        m_mode = (m_mode+1)%6;

        update();
    }
}

void GLWidget::wheelEvent(QWheelEvent *event)
{
    double ds = 0.05;
    if(event->delta()>0 && scale-ds>0)
        scale-=ds;
    else if(event->delta()<0)
        scale+=ds;
    update();
}

void GLWidget::slot_timerRoaming()
{
    m_horAngle+=2;
    this->update();
}

void GLWidget::slot_timerSaveCaptures()
{
    static int curCount = 0;

    curCount++;
    if(curCount>this->m_captureAmount)
    {
        this->m_timer.stop();
        curCount = 0;
        return;
    }

    m_horAngle+=(360.0/this->m_captureAmount);

    m_isReadDepthBuffer = true;
    this->update();

//    QFile file("C:/Users/LiewDennis/Desktop/depthBuffer.txt");

//    if(!file.open(QIODevice::WriteOnly))
//        return;

//    QTextStream ts(&file);

//    int bufferSize = width()*height();

//    for(unsigned int i=0; i<bufferSize; i++)
//    {
//        ts<<m_depthBuffer[i]<<"\n";
//    }

//    file.close();

    if(!this->m_depthBuffer)
        return;

    QImage image(width(),height(),QImage::Format_ARGB32);

    float min = 9999;
    float max = -1;
    int minx = 1000999;
    int miny = 10000999;
    int maxx = -1;
    int maxy = -1;
    for(int x=0; x<width(); x++)
    {
        for(int y=0 ; y<height(); y++)
        {
            if(m_depthBuffer[y*width()+x] == 1)
                continue;
            if(m_depthBuffer[y*width()+x] < min)
                min = m_depthBuffer[y*width()+x];
            if(m_depthBuffer[y*width()+x] > max)
                max = m_depthBuffer[y*width()+x];

            if(minx > x)
                minx = x;
            if(miny > y)
                miny = y;
            if(maxx < x)
                maxx = x;
            if(maxy < y)
                maxy = y;
        }
    }

    float diff = max-min;
//    qDebug()<<max<<min<<diff;
//    qDebug()<<"x"<<minx<<maxx;
//    qDebug()<<"y"<<miny<<maxy;
    if(diff==0)
    {
        qDebug()<<"[ERROR] DIFF=0";
        return;
    }
    if(maxx<=minx || maxy<=miny)
    {
        qDebug()<<"[ERROR] CAN'T FORM A RECTANGLE!!";
        return;
    }
    for(unsigned int x=0; x<width(); x++)
    {
        for(unsigned int y=0 ; y<height(); y++)
        {
            float gray = 255*(m_depthBuffer[y*width()+x]-min)/diff;


            if(m_depthBuffer[y*width()+x] == 1)
                gray = 255;
            //qDebug()<<gray;
//            if(gray!=255)
//                qDebug()<<gray;
            image.setPixel(x,y,qRgb(gray,gray,gray));
        }
    }
    image = image.copy(minx,miny,fabs(maxx-minx),fabs(maxy-miny));
    image = image.mirrored(false,true);

    image.save(QString(m_captureDir+"/depthBuffer_%1.png").arg(curCount));

    emit sig_CurCaptureCount(curCount,this->m_captureAmount);
}


GLWidget::~GLWidget(){}
