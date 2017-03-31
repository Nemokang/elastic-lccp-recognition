/*
 *直接分割深度图 * 数据类型是ununsigned short *
 */

#include "Segmentation.h"



Segmentation::Segmentation()
    :vertexTexture(Resolution::getInstance().width(),
                   Resolution::getInstance().height(),
                   GL_RGBA32F,
                   GL_LUMINANCE,
                   GL_FLOAT,
                   false,
                   true),
     normalTexture(Resolution::getInstance().width(),
                   Resolution::getInstance().height(),
                   GL_RGBA32F,
                   GL_LUMINANCE,
                   GL_FLOAT,
                   false,
                   true),
     vertexProgram(loadProgramFromFile("empty.vert", "Segmentation_vertex.frag", "quad.geom")),
     vertexRenderBuffer(Resolution::getInstance().width(), Resolution::getInstance().height()),
     normalProgram(loadProgramFromFile("empty.vert", "Segmentation_normal.frag", "quad.geom")),
     normalRenderBuffer(Resolution::getInstance().width(), Resolution::getInstance().height())

{
    vertexFrameBuffer.AttachColour(*vertexTexture.texture);
    vertexFrameBuffer.AttachDepth(vertexRenderBuffer);

    normalFrameBuffer.AttachColour(*normalTexture.texture);
    normalFrameBuffer.AttachDepth(normalRenderBuffer);

//    cols=(float)Resolution::getInstance().cols();//640
//    rows=(float)Resolution::getInstance().rows();//480


//    cam = SEGVEC4(Intrinsics::getInstance().cx(),
//                  Intrinsics::getInstance().cy(),
//                  1.0 / Intrinsics::getInstance().fx(),
//                  1.0 / Intrinsics::getInstance().fy());


//    //计算每个像素点对应的二维纹理坐标
//    for(int i = 0; i < Resolution::getInstance().width(); i++)
//    {
//        for(int j = 0; j < Resolution::getInstance().height(); j++)
//        {
//            texcoords.push_back(SEGVEC2(((float)i / (float)Resolution::getInstance().width()) + 1.0 / (2 * (float)Resolution::getInstance().width()),
//                                   ((float)j / (float)Resolution::getInstance().height()) + 1.0 / (2 * (float)Resolution::getInstance().height())));
//        }
//    }
}

Segmentation::~Segmentation()
{

}

void Segmentation::vertex(GPUTexture * rawDepth,const Eigen::Matrix4f & pose ,int first)
{
    vertexFrameBuffer.Bind();

    glPushAttrib(GL_VIEWPORT_BIT);

    glViewport(0, 0, vertexRenderBuffer.width, vertexRenderBuffer.height);

    glClearColor(0, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    vertexProgram->Bind();

    vertexProgram->setUniform(Uniform("rSampler", 0));
    vertexProgram->setUniform(Uniform("pose", pose));

    Eigen::Vector4f cam(Intrinsics::getInstance().cx(),
                  Intrinsics::getInstance().cy(),
                  1.0f / Intrinsics::getInstance().fx(),
                  1.0f / Intrinsics::getInstance().fy());

    vertexProgram->setUniform(Uniform("cam", cam));
    vertexProgram->setUniform(Uniform("cols", (float)Resolution::getInstance().cols()));
    vertexProgram->setUniform(Uniform("rows", (float)Resolution::getInstance().rows()));
        vertexProgram->setUniform(Uniform("first", first));

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, rawDepth->texture->tid);

    glDrawArrays(GL_POINTS, 0, 1);

    vertexFrameBuffer.Unbind();

    glBindTexture(GL_TEXTURE_2D, 0);

    glActiveTexture(GL_TEXTURE0);

    vertexProgram->Unbind();

    glPopAttrib();

    glFinish();
}

void Segmentation::normal(GPUTexture * rawDepth, const Eigen::Matrix4f & pose,int first)
{
    normalFrameBuffer.Bind();

    glPushAttrib(GL_VIEWPORT_BIT);

    glViewport(0, 0, normalRenderBuffer.width, normalRenderBuffer.height);

    glClearColor(0, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    normalProgram->Bind();

    normalProgram->setUniform(Uniform("rSampler", 0));
    normalProgram->setUniform(Uniform("pose", pose));

    Eigen::Vector4f cam(Intrinsics::getInstance().cx(),
                  Intrinsics::getInstance().cy(),
                  1.0f / Intrinsics::getInstance().fx(),
                  1.0f / Intrinsics::getInstance().fy());

    normalProgram->setUniform(Uniform("cam", cam));
    normalProgram->setUniform(Uniform("cols", (float)Resolution::getInstance().cols()));
    normalProgram->setUniform(Uniform("rows", (float)Resolution::getInstance().rows()));
    normalProgram->setUniform(Uniform("first", first));

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, rawDepth->texture->tid);

    glDrawArrays(GL_POINTS, 0, 1);

    normalFrameBuffer.Unbind();

    glBindTexture(GL_TEXTURE_2D, 0);

    glActiveTexture(GL_TEXTURE0);

    normalProgram->Unbind();

    glPopAttrib();

    glFinish();
}


//labelColor[(i*640+j)*3]=(normBuff.at<Eigen::Vector4f>(i, j)(0)+1.0)*0.5*255.0;
//labelColor[(i*640+j)*3+1]=(normBuff.at<Eigen::Vector4f>(i, j)(1)+1.0)*0.5*255.0;
//labelColor[(i*640+j)*3+2]=(normBuff.at<Eigen::Vector4f>(i, j)(2)+1.0)*0.5*255.0;

//根据labe图返回rgb图
void Segmentation::getLabel( Img<Eigen::Vector4f> rawVertexBuff, Img<Eigen::Vector4f> normBuff ,int*label)
{
 //   int label[480*640];

    int l=1;

    for(int i=0;i<480;i++)
    {
        for(int j=0;j<640;j++)
        {//std::cout<<l<<" ";
            if(fabs(normBuff.at<Eigen::Vector4f>(i,j)(0))<0.00001 && fabs(normBuff.at<Eigen::Vector4f>(i,j)(1))<0.00001 && fabs(normBuff.at<Eigen::Vector4f>(i,j)(2))<0.00001 )
            {
                label[i*640+j]= 0;
                continue;
            }

            SegNode q(rawVertexBuff.at<Eigen::Vector4f>(i,j),normBuff.at<Eigen::Vector4f>(i,j));

            if(i==0 && j==0)
            {
                label[0]=l;
            }
            else if(i==0 && j>0)
            {
                SegNode p(rawVertexBuff.at<Eigen::Vector4f>(i,j-1),normBuff.at<Eigen::Vector4f>(i,j-1));

                if(ifconvex(p,q))
                {label[j]=label[j-1];}
                else
                {
                    l++;
                    label[j]= l ;
                }
            }
            else if(j==0 && i>0)
            {
                SegNode p(rawVertexBuff.at<Eigen::Vector4f>(i-1,j),normBuff.at<Eigen::Vector4f>(i-1,j));
                //SegNode q(rawVertexBuff.at<Eigen::Vector4f>(i,j),normBuff.at<Eigen::Vector4f>(i,j));
                if(ifconvex(p,q))
                {label[i*640]=label[(i-1)*640];}
                else
                {
                    l++;
                    label[i*640]= l ;
                }
            }
            else
            {

                SegNode p1(rawVertexBuff.at<Eigen::Vector4f>(i-1,j),normBuff.at<Eigen::Vector4f>(i-1,j));
                SegNode p2(rawVertexBuff.at<Eigen::Vector4f>(i,j-1),normBuff.at<Eigen::Vector4f>(i,j-1));
                //SegNode q(rawVertexBuff.at<Eigen::Vector4f>(i,j),normBuff.at<Eigen::Vector4f>(i,j));
                if(ifconvex(p1,q))
                {label[i*640+j]=label[(i-1)*640+j]; continue;}
                if(ifconvex(p2,q))
                {label[i*640+j]=label[i*640+j-1]; continue;}
                if(!ifconvex(p1,q) && !ifconvex(p2,q))
                {
                    l++;
                    label[i*640+j]= l ;
                }
            }
        }
       // std::cout<<std::endl;
    }
}

bool Segmentation::ifconvex(SegNode p ,SegNode q)
{
    Eigen::Vector3f d(p.pointvertex(0)-q.pointvertex(0),
                      p.pointvertex(1)-q.pointvertex(1),
                      p.pointvertex(2)-q.pointvertex(2));

    Eigen::Vector3f n(p.pointnormal(0)-q.pointnormal(0),
                      p.pointnormal(1)-q.pointnormal(1),
                      p.pointnormal(2)-q.pointnormal(2));

   return bool( d(0)*n(0)+d(1)*n(1)+d(2)*n(2));
}

//Forward difference on raw depth maps still in ushort mm
//Cam is //cx, cy, 1 / fx, 1 / fy
//dataProgram->setUniform(Uniform("cam", Eigen::Vector4f(Intrinsics::getInstance().cx(),
//                                                 Intrinsics::getInstance().cy(),
//                                                 1.0 / Intrinsics::getInstance().fx(),
//                                                 1.0 / Intrinsics::getInstance().fy())));

//dataProgram->setUniform(Uniform("cols", (float)Resolution::getInstance().cols()));
//dataProgram->setUniform(Uniform("rows", (float)Resolution::getInstance().rows()));
//dataProgram->setUniform(Uniform("pose", pose));


/*
SEGVEC3 Segmentation::getVertex(float x, float y, unsigned short depth, SEGVEC4 cam)
{
    float z = depth;
    return SEGVEC3((x - cam.x) * z * cam.z, (y - cam.y) * z * cam.w, z);//统一把vec3的形式改了
}

////Cam is //cx, cy, 1 / fx, 1 / fy
SEGVEC3 Segmentation::getNormal( float x, float y,unsigned short *depth,SEGVEC4 cam)//暂时不用pose进行转置
{
    SEGVEC3 vPosition = getVertex(x, y, depth[0], cam);

    /////////注意点坐标和深度值的对应
    SEGVEC3 vPosition_xf = getVertex(x + 1, y, depth[2], cam);
    SEGVEC3 vPosition_xb = getVertex(x - 1, y, depth[1], cam);

    SEGVEC3 vPosition_yf = getVertex(x, y + 1, depth[4], cam);
    SEGVEC3 vPosition_yb = getVertex(x, y - 1, depth[3], cam);

//    SEGVEC3 del_x = ((vPosition_xb + vPosition) / 2) - ((vPosition_xf + vPosition) / 2);
//    SEGVEC3 del_y = ((vPosition_yb + vPosition) / 2) - ((vPosition_yf + vPosition) / 2);

    SEGVEC3 del_x = ((vPosition_xb + vPosition) / 2) - ((vPosition_xf + vPosition) / 2);
    SEGVEC3 del_y = ((vPosition_yb + vPosition) / 2) - ((vPosition_yf + vPosition) / 2);

    SEGVEC3 norm = del_x.cross(del_y);
    norm.normalize();
    return norm;
}

void Segmentation::getLabelRGB( Img<Eigen::Vector4f> consBuff, unsigned short * normTarget,Eigen::Matrix4f currPose)
{

    for(int i = 0; i < consBuff.cols; i++)
    {
        for(int j = 0; j < consBuff.rows; j++)
        {
            if(consBuff.at<Eigen::Vector4f>(j, i)(2) > 0 &&
               consBuff.at<Eigen::Vector4f>(j, i)(2) < 255 )
            {
                Eigen::Vector4f worldRawPoint = currPose * Eigen::Vector4f(consBuff.at<Eigen::Vector4f>(j, i)(0),
                                                                           consBuff.at<Eigen::Vector4f>(j, i)(1),
                                                                           consBuff.at<Eigen::Vector4f>(j, i)(2),
                                                                           1.0f);

                normTarget[j*640+i]= worldRawPoint(0);
            }
        }
    }
}
*/

/*
void Segmentation::getLabelRGB( Img<Eigen::Vector4f> consBuff,unsigned char * depthTarget,Eigen::Matrix4f currPose)
{
    const int size = (int)(cols*rows);
    //const int sourceSize = (int)(640*480*2);//默认这是*depthSource指针的大小
    //const int targetSize = (int)(640*480*3);//默认这是*depthTarget指针的大小

//    std::cout<<depthSource[0]<<","<<depthSource[1]<<std::endl;
//    std::cout<<depthSource[2]<<","<<depthSource[3]<<std::endl;
//    std::cout<<depthSource[4]<<","<<depthSource[5]<<std::endl;

    std::vector<SegNode> labelNodes;

   //SEGVEC3 vTest(0,0,1);
    ///
    ///这个循环计算每个像素点的法向量
    ///
    for(int i=0;i<size;i++)//循环计算传入的深度图的每个点的法向量=====深度图是６４０×４８０×２,只用一半像素做运算
   {
//        if(i/(int)cols==0 || i/(int)cols==(int)(cols-1) || i%(int)cols==0 || i%(int)(cols-1))// 由于计算法向量分别用到了点（x-1,y)(x+1,y)(x,y-1)(x,y+1),所以边缘的像素不能用于计算法向量
//        {//std::cout<<"("<<i/(int)cols<<","<<i%(int)cols<<")"<<std::endl;
//            continue;
//        }
//        else
//        {
            //计算i点的法向量
            SEGVEC2 texcoord=texcoords[i];//第i个点在２Ｄ图上的x,y坐标----初始化时计算
            float x = float(texcoord.x * cols);
            float y = float(texcoord.y * rows);

//            unsigned short  depthforNorm[5];
//            depthforNorm[0]=(depthSource[i*2]*2+depthSource[i*2+1]);
//            depthforNorm[1]=(depthSource[(i-1)*2]+depthSource[(i-1)*2+1]);
//            depthforNorm[2]=(depthSource[(i+1)*2]+depthSource[(i+1)*2+1]);
//            depthforNorm[3]=(depthSource[(i-(int)cols)*2]+depthSource[(i-(int)cols)*2+1]);
//            depthforNorm[4]=(depthSource[(i+(int)cols)*2]+depthSource[(i+(int)cols)*2+1]);

//            SEGVEC3 vecNorm = getNormal(x,y, depthforNorm,cam)*currPose;

            SEGVEC3 vPosition = getVertex(x, y, depthSource[i], cam);

            SegNode labelNode;//将i点转化成一个node
            labelNode.i=i;
//            labelNode.vecNormal=vecNorm;
            labelNodes.push_back(labelNode);

//            depthTarget[i*3]=(vPosition.y/3000.0)*255;
//            depthTarget[i*3+1]=0;
//            depthTarget[i*3+2]=0;
                depthTarget[i*3]=(1.0-(float)depthSource[i]/3000.0)*255;
                depthTarget[i*3+1]=(1.0-(float)depthSource[i]/3000.0)*255;
                depthTarget[i*3+2]=(1.0-(float)depthSource[i]/3000.0)*255;

            //std::cout<<(int)depthTarget[i*3]<<","<<(int)depthTarget[i*3+1]<<","<<(int)depthTarget[i*3+2]<<std::endl;
            //std::cout<<vPosition.x<<","<<vPosition.y<<","<<vPosition.z<<std::endl;
            //std::cout<<depthSource[i*2]<<std::endl;
            //std::cout<<"--------------------"<<std::endl;

 //       }

    }

    //std::cout<<"**"<<labelNodes[0].vecNormal<<"**"<<std::endl;

//    ///这个循环计算和第一个像素点法向量乘积>0的点
//     labelNodes[1].label=1;
//    for(int i=1;i<size;i++)//循环计算传入的深度图的每个点的法向量=====深度图是６４０×４８０×２,只用一半像素做运算
//   {
//        //计算i点的法向量
//        if(labelNodes[1].vecNormal.potProdect(labelNodes[i].vecNormal)<0)
//        {
//            labelNodes[i].label=1;
//            //std::cout<<labelNodes[1].label;
//        }
//        else
//        {
//            labelNodes[i].label=0;
//            //std::cout<<labelNodes[i].label;
//        }
//    }


//     for(int i=0;i<size;i++)
//     {
//         if(labelNodes[i].label==1)
//         {
//             depthTarget[i*3]=100;
//             depthTarget[i*3+1]=0;
//             depthTarget[i*3+2]=0;
//         }
//         else
//         {
//             depthTarget[i*3]=0;
//             depthTarget[i*3+1]=100;
//             depthTarget[i*3+2]=0;
//         }
//     }

}
*/

