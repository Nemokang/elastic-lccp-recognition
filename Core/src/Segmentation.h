/*
 *直接分割深度图 *
 */
#ifndef SEGMENTATION_H_
#define SEGMENTATION_H_

//#include "Shaders/Uniform.h"
#include "Utils/Resolution.h"
#include "Utils/Intrinsics.h"
#include "SegVec.h"//----------------------------自定义vec类
#include <Eigen/Core>
#include <stdio.h>
//#include <Eigen/Geometry>//----------为了算叉积
#include <pangolin/gl/gl.h>

#include "Utils/Img.h"

//#include <Eigen/Dense>
//using namespace Eigen;

#include "Shaders/Shaders.h"
#include "GPUTexture.h"

#include <vector>


typedef SegVec2 SEGVEC2;
typedef SegVec3 SEGVEC3;
typedef SegVec4 SEGVEC4;

class Segmentation
{
    public:
        Segmentation();
        virtual ~Segmentation();

        class SegNode
        {
            public:
                SegNode()
                {}
                SegNode(Eigen::Vector4f pntvertex,Eigen::Vector4f pntnormal)
                {
                    pointvertex=pntvertex;
                    pointnormal=pntnormal;
                }

                Eigen::Vector4f pointvertex;
                Eigen::Vector4f pointnormal;

//                int i; //node在图像数据流中的index
//                int label;//标签值
//                SEGVEC3 vecNormal;//每个点的法向量
        };


        void normal(GPUTexture * rawDepth, const Eigen::Matrix4f & pose,int first);//-------------
        void vertex(GPUTexture * rawDepth,const Eigen::Matrix4f & pose,int first);

        GPUTexture vertexTexture;
        GPUTexture normalTexture;

        std::shared_ptr<Shader> vertexProgram;
        pangolin::GlRenderBuffer vertexRenderBuffer;
        pangolin::GlFramebuffer vertexFrameBuffer;

        std::shared_ptr<Shader> normalProgram;
        pangolin::GlRenderBuffer normalRenderBuffer;
        pangolin::GlFramebuffer normalFrameBuffer;

        unsigned char colorType[30][3]={{255,218,185},
                                        {0,205,205},
                                        {25,25,112},
                                        {46,139,87},
                                        {202,255,112},
                                        {106,90,205},
                                        {205,173,0},
                                        {255,20,147},
                                        {0,200,50},
                                        {100,0,255},

                                        {255,218,85},
                                         {50,205,205},
                                         {25,25,11},
                                         {46,39,87},
                                         {102,255,12},
                                         {106,200,205},
                                         {205,173,50},
                                         {55,20,147},
                                         {0,150,50},
                                         {100,25,255},

                                        {55,218,85},
                                         {150,205,205},
                                         {125,25,11},
                                         {146,39,87},
                                         {42,255,12},
                                         {56,200,205},
                                         {25,173,50},
                                         {155,20,147},
                                         {50,150,50},
                                         {0,25,255}};

        //根据labe图返回rgb图
        void getLabel( Img<Eigen::Vector4f> rawVertexBuff, Img<Eigen::Vector4f> normBuff ,int*label);
        bool ifconvex(SegNode p ,SegNode q);
        //void label(Img<Eigen::Vector4f> rawVertexBuff, Img<Eigen::Vector4f> normBuff,SegNode p,int ii,int jj,int* label);


        //Forward difference on raw depth maps still in ushort mm
        //Cam is //cx, cy, 1 / fx, 1 / fy
//        SEGVEC3 getVertex(float x, float y, unsigned short depth, SEGVEC4 cam);

//        //Cam is //cx, cy, 1 / fx, 1 / fy
//        SEGVEC3 getNormal(float x, float y,unsigned short *depth,SEGVEC4 cam);
        //返回一张label图Eigen::
        //int getLabel();


        //int Segmentation::labelofNode(std::vector<SegNode> labelNodes);//获取每个点的标签值




//    private:
//        float cols;
//        float rows;

//        SEGVEC4 cam;//相机参数

//        std::vector<SEGVEC2> texcoords;//每个像素点对应的二维纹理坐标

};


#endif
