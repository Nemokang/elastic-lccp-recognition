#include "SegVec.h"

//u x v = { u2v3-v2u3 , u3v1-v3u1 , u1v2-u2v1 }叉积公式
SegVec3 SegVec3::cross(SegVec3 v)//当前向量与参数中向量的叉积
{
    SegVec3 tempVector;//未归一化的法向量
    tempVector.x=y*v.z - v.y*z;
    tempVector.y=z*v.x - v.z*x;
    tempVector.z=x*v.y - y*v.x;

    return tempVector;
}

float SegVec3::potProdect(SegVec3 v)//当前向量与参数中向量的点积
{
     return float(x*v.x + y*v.y +z*v.z);
}

void SegVec3::normalize()//当前向量归一化
{
    float vectorlen = (float)sqrt(x*x+y*y+z*z);
    if(vectorlen>0)
    {
        x = x/vectorlen;
        y = y/vectorlen;
        z = z/vectorlen;
    }
}


SegVec3 SegVec3::operator * (Eigen::Matrix4f mat4)
{
//    Eigen::Matrix3f mat3;
//    mat3 << mat4(0,0), mat4(0,1), mat4(0,2),
//            mat4(1,0), mat4(1,1), mat4(1,2),
//            mat4(2,0), mat4(2,1), mat4(2,2);

//    return SegVec3(x*mat3(0,0)+y*mat3(0,1)+z*mat3(0,2) ,
//                   x*mat3(1,0)+y*mat3(1,1)+z*mat3(1,2) ,
//                   x*mat3(2,0)+y*mat3(2,1)+z*mat3(2,2) );

    return SegVec3( x*mat4(0,0)+y*mat4(0,1)+z*mat4(0,2)+mat4(0,3) ,
                    x*mat4(1,0)+y*mat4(1,1)+z*mat4(1,2)+mat4(1,3) ,
                    x*mat4(2,0)+y*mat4(2,1)+z*mat4(2,2)+mat4(2,3) );
}




