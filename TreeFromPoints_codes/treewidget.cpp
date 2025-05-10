#include "treewidget.h"
#define VERTEX_ATTRIBUTE  0
#define COLOUR_ATTRIBUTE  1
#define NORMAL_ATTRIBUTE  2
#define TEXTURE_ATTRIBUTE 3

TreeWidget::TreeWidget(QWidget *parent) :
    QOpenGLWidget(parent)
{
    this->setFocusPolicy(Qt::StrongFocus);

    m_barkTexture = NULL;
    m_leafTexture = NULL;
    m_meshProgram = NULL;
    m_leafProgram = NULL;
    m_skeletonProgram = NULL;
    m_pointProgram = NULL;

    m_polygonMode = true;
    m_isBarkTextured = true;
    m_isShowSeg = false;

    m_projectMode = _Perspective;
    m_horAngle = 45.0f;
    m_verAngle = 0.0f;
    distance = 500.0f;
    scale = 1.0f;
    m_eyeDist = QVector3D(0.0f,0.0f,0.0f);

    m_tree = new TreeModeler();

    connect(&m_viewingTimer,SIGNAL(timeout()),this,SLOT(OnViewingTimer()));
}


void TreeWidget::initShaders(QOpenGLShaderProgram*& m_program,const QString& shaderName,
                           bool v = false,bool c = false, bool n = false, bool t=false)
{
    m_program = new QOpenGLShaderProgram(this);

    m_program->addShaderFromSourceFile(QOpenGLShader::Vertex,  QString(":/3dmesh/res/shaders/3DMesh/%1VShader.glsl").arg(shaderName));
    m_program->addShaderFromSourceFile(QOpenGLShader::Fragment,QString(":/3dmesh/res/shaders/3DMesh/%1FShader.glsl").arg(shaderName));


    m_program->link();
    m_program->bind();
}


void TreeWidget::setupShaders(QOpenGLShaderProgram *&m_program)
{
    m_program->bind();

    m_program->setUniformValue("mat_projection",m_projectMatrix);
    m_program->setUniformValue("mat_view",m_viewMatrix);
}

void TreeWidget::setupTexture(QOpenGLTexture *&texture, const QString &filename)
{
    if(texture)
        delete texture;

    QImage img(filename);
    texture = new QOpenGLTexture(img);

    texture->setMinificationFilter(QOpenGLTexture::Nearest);
    texture->setMagnificationFilter(QOpenGLTexture::Linear);
}

void TreeWidget::SetLeafTexture(const QString &filename)
{
    setupTexture(m_leafTexture,filename);
}

void TreeWidget::SetBarkTexture(const QString& filename)
{
    setupTexture(m_barkTexture,filename);
}


void TreeWidget::initializeGL()
{


    glEnable(GL_BLEND);
    glEnable(GL_ALPHA_TEST);
    glEnable(GL_TEXTURE_2D);

    glClearColor(0.997, 0.947, 0.947,1.0f);


    initShaders(m_meshProgram,"bark",true,false,true,true);
    initShaders(m_leafProgram,"Leaf",true,false,true,true);

    setupTexture(m_leafTexture,":/3dmesh/res/shaders/3DMesh/default_leaf.png");
    setupTexture(m_barkTexture,":/3dmesh/res/shaders/3DMesh/default_bark.png");  // 0default  shurb_1    shurb_1红色    shurb_2
    qsrand(QTime::currentTime().msec()*QTime::currentTime().second());


}

void TreeWidget::resizeGL(int w, int h)
{
    glViewport(0,0,width(),height());

    m_projectMatrix.setToIdentity();

    if(m_projectMode == _Perspective)
        m_projectMatrix.perspective(45.0f,(float)w/(float)h,0.1f,1000.0f);
    else if(m_projectMode == _Ortho)
        m_projectMatrix.ortho(-400,400,-400,400,-100.0,1000);
}

void TreeWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


    if(m_projectMode == _Perspective)
    {
        m_eyePos = m_eyeDist+QVector3D(scale*distance*cos(PI*m_verAngle/180.0)*cos(PI*m_horAngle/180.0),
                                       scale*distance*sin(PI*m_verAngle/180.0),
                                       scale*distance*cos(PI*m_verAngle/180.0)*sin(PI*m_horAngle/180.0));
        m_viewMatrix.setToIdentity();
        m_viewMatrix.lookAt(m_eyePos,m_eyeDist,QVector3D(0,1,0));
        m_modelMat.setToIdentity();
    }
    else if(m_projectMode == _Ortho)
    {
       m_viewMatrix.setToIdentity();
       m_modelMat.setToIdentity();
    }


    if(!m_polygonMode)
        glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
    else
        glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);

    {
        // tree
        setupShaders(m_meshProgram);
        glEnable(GL_TEXTURE0);
        m_meshProgram->setUniformValue("texture",0);
        m_barkTexture->bind();
        m_tree->DrawTreeMeshVBO(m_meshProgram,m_modelMat);
    }
}

void TreeWidget::mousePressEvent(QMouseEvent *event)
{
    m_clickpos = event->pos();
}

void TreeWidget::mouseMoveEvent(QMouseEvent *event)
{
    if(event->buttons() == Qt::LeftButton)
    {
        QPoint cur = event->pos();

        double dx = (cur.x() - m_clickpos.x())/5.0;
        double dy = (cur.y() - m_clickpos.y())/5.0;

        m_horAngle +=dx;
        if(m_verAngle+dy <90.0 && m_verAngle+dy>-90.0)
            m_verAngle +=dy;

        m_clickpos = cur;
        update();
    }
}

void TreeWidget::mouseReleaseEvent(QMouseEvent *event)
{
}

void TreeWidget::keyPressEvent(QKeyEvent *e)
{
    if(e->key() == Qt::Key_W)
        m_eyeDist.setY(m_eyeDist.y()+5);
    if(e->key() == Qt::Key_S)
        m_eyeDist.setY(m_eyeDist.y()-5);
    if(e->key() == Qt::Key_C)
        m_polygonMode = ! m_polygonMode;
    this->update();
}

void TreeWidget::wheelEvent(QWheelEvent *event)
{
    double ds = 0.03;
    if(event->delta()>0 && scale-ds>0)
        scale-=ds;
    else if(event->delta()<0)
        scale+=ds;
    update();
}

void TreeWidget::OnViewingTimer()
{
    m_horAngle +=1;
    update();
}

TreeWidget::~TreeWidget(){}
