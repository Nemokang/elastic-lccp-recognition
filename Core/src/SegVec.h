#include <math.h>
#include <Eigen/Core>

class SegVec2
{
    public:
        SegVec2(){}
        SegVec2(float x0,float y0){x=x0;y=y0;}
        float x;
        float y;

        //重载运算符
        SegVec2 operator + (SegVec2 v)//声明运算符的"+"函数
        {
            return SegVec2(x+v.x,y+v.y);
        }
        SegVec2 operator - (SegVec2 v)//声明运算符的"-"函数
        {
            return SegVec2(x-v.x,y-v.y);
        }
};
class SegVec3
{
    public:
        SegVec3()
        {}
        SegVec3(float x0,float y0,float z0){x=x0;y=y0;z=z0;}
        float x;
        float y;
        float z;

        float potProdect(SegVec3 v);//当前向量与参数中向量的点积
        SegVec3 cross(SegVec3 v);//当前向量与参数中向量的叉积
        void normalize();//当前向量归一化后返回

        //重载运算符
        SegVec3 operator + (SegVec3 v)//声明运算符的"+"函数
        {
            return SegVec3(x+v.x, y+v.y, z+v.z);
        }
        SegVec3 operator - (SegVec3 v)//声明运算符的"-"函数
        {
            return SegVec3(x-v.x, y-v.y, z-v.z);
        }
        SegVec3 operator / (int n)//声明运算符的"/"函数
        {
            return SegVec3(x/n, y/n, z/n);
        }
        SegVec3 operator * (Eigen::Matrix4f mat4);//声明运算符的"/"函数

};
class SegVec4
{
    public:
        SegVec4()
        {}
        SegVec4(float x0,float y0,float z0,float w0){x=x0;y=y0;z=z0;w=w0;}
        float x;
        float y;
        float z;
        float w;

        //重载运算符
        SegVec4 operator + (SegVec4 v)//声明运算符的"+"函数
        {
            return SegVec4(x+v.x, y+v.y, z+v.z, w+v.w);
        }
        SegVec4 operator - (SegVec4 v)//声明运算符的"-"函数
        {
            return SegVec4(x-v.x, y-v.y, z-v.z, w-v.w);
        }
};
