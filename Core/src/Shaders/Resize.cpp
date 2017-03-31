/*
 * This file is part of ElasticFusion.
 *
 * Copyright (C) 2015 Imperial College London
 * 
 * The use of the code within this file and all code within files that 
 * make up the software that is ElasticFusion is permitted for 
 * non-commercial purposes only.  The full terms and conditions that 
 * apply to the code within this file are detailed within the LICENSE.txt 
 * file and at <http://www.imperial.ac.uk/dyson-robotics-lab/downloads/elastic-fusion/elastic-fusion-license/> 
 * unless explicitly stated.  By downloading this file you agree to 
 * comply with these terms.
 *
 * If you wish to use any of this code for commercial purposes then 
 * please email researchcontracts.engineering@imperial.ac.uk.
 *
 */

#include "Resize.h"

Resize::Resize(int srcWidth,
               int srcHeight,
               int destWidth,
               int destHeight)
: imageTexture(destWidth,
               destHeight,
               GL_RGBA,
               GL_RGB,
               GL_UNSIGNED_BYTE,
               false,
               true),
  vertexTexture(destWidth,
                destHeight,
                GL_RGBA32F,
                GL_LUMINANCE,
                GL_FLOAT,
                false,
                true),
 normalTexture(srcWidth,//-------------------------------new add
                srcHeight,
                GL_RGBA32F,
                GL_LUMINANCE,
                GL_FLOAT,
                false,
                true),
  timeTexture(destWidth,
              destHeight,
              GL_LUMINANCE16UI_EXT,
              GL_LUMINANCE_INTEGER_EXT,
              GL_UNSIGNED_SHORT,
              false,
              true),
  rawVertexTexture(srcWidth,//-------------------------------new add------------------------------
              srcHeight,
                   GL_RGBA32F,
                   GL_LUMINANCE,
                   GL_FLOAT,
                   false,
                   true),
  imageProgram(loadProgramFromFile("empty.vert", "resize.frag", "quad.geom")),
  imageRenderBuffer(destWidth, destHeight),
  vertexProgram(loadProgramFromFile("empty.vert", "resize.frag", "quad.geom")),
  vertexRenderBuffer(destWidth, destHeight),
  normalProgram(loadProgramFromFile("empty.vert", "resize.frag", "quad.geom")),//---------------new add
  normalRenderBuffer(srcWidth, srcHeight),//---------------new add
  timeProgram(loadProgramFromFile("empty.vert", "resize.frag", "quad.geom")),
  timeRenderBuffer(destWidth, destHeight),
  rawVertexProgram(loadProgramFromFile("empty.vert", "resize.frag", "quad.geom")),//---------------new add----------------
  rawVertexRenderBuffer(srcWidth, srcHeight)//---------------new add-----------------------
{
   imageFrameBuffer.AttachColour(*imageTexture.texture);
   imageFrameBuffer.AttachDepth(imageRenderBuffer);

   vertexFrameBuffer.AttachColour(*vertexTexture.texture);
   vertexFrameBuffer.AttachDepth(vertexRenderBuffer);

   normalFrameBuffer.AttachColour(*normalTexture.texture);//---------------new add
   normalFrameBuffer.AttachDepth(normalRenderBuffer);//--------------new add

   timeFrameBuffer.AttachColour(*timeTexture.texture);
   timeFrameBuffer.AttachDepth(timeRenderBuffer);

   rawVertexFrameBuffer.AttachColour(*rawVertexTexture.texture);//---------------new add-------------------
   rawVertexFrameBuffer.AttachDepth(rawVertexRenderBuffer);//--------------new add-----------------

}

Resize::~Resize()
{
}

void Resize::image(GPUTexture * source, Img<Eigen::Matrix<unsigned char, 3, 1>> & dest)
{
    imageFrameBuffer.Bind();

    glPushAttrib(GL_VIEWPORT_BIT);

    glViewport(0, 0, imageRenderBuffer.width, imageRenderBuffer.height);

    glClearColor(0, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    imageProgram->Bind();

    imageProgram->setUniform(Uniform("eSampler", 0));//设置eSampler的location为０

    glActiveTexture(GL_TEXTURE0);//使纹理单元GL_TEXTURE0成为活动状态
    glBindTexture(GL_TEXTURE_2D, source->texture->tid);//将纹理对象绑定到活跃的纹理单元上

    glDrawArrays(GL_POINTS, 0, 1);

    glReadPixels(0, 0, imageRenderBuffer.width, imageRenderBuffer.height, GL_RGB, GL_UNSIGNED_BYTE, dest.data);//将图像数据存放到dest中

    imageFrameBuffer.Unbind();

    glBindTexture(GL_TEXTURE_2D, 0);

    glActiveTexture(GL_TEXTURE0);

    imageProgram->Unbind();

    glPopAttrib();

    glFinish();
}

void Resize::vertex(GPUTexture * source, Img<Eigen::Vector4f> & dest )
{
    vertexFrameBuffer.Bind();

    glPushAttrib(GL_VIEWPORT_BIT);

    glViewport(0, 0, vertexRenderBuffer.width, vertexRenderBuffer.height);

    glClearColor(0, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    vertexProgram->Bind();

    vertexProgram->setUniform(Uniform("eSampler", 0));

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, source->texture->tid);

    glDrawArrays(GL_POINTS, 0, 1);

    glReadPixels(0, 0, vertexRenderBuffer.width, vertexRenderBuffer.height, GL_RGBA, GL_FLOAT, dest.data);

    vertexFrameBuffer.Unbind();

    glBindTexture(GL_TEXTURE_2D, 0);

    glActiveTexture(GL_TEXTURE0);

    vertexProgram->Unbind();

    glPopAttrib();

    glFinish();
}

void Resize::normal_noDownSampling(GPUTexture * source, Img<Eigen::Vector4f> & dest )//-----------------new add
{
    normalFrameBuffer.Bind();

    glPushAttrib(GL_VIEWPORT_BIT);

    //glViewport(0, 0, normalRenderBuffer.width, normalRenderBuffer.height);
    glViewport(0, 0, 640, 480);

    glClearColor(0, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    normalProgram->Bind();

    normalProgram->setUniform(Uniform("eSampler", 0));

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, source->texture->tid);

    glDrawArrays(GL_POINTS, 0, 1);

    //glReadPixels(0, 0, normalRenderBuffer.width, normalRenderBuffer.height, GL_RGBA, GL_FLOAT, dest.data);
    glReadPixels(0, 0, 640, 480, GL_RGBA, GL_FLOAT, dest.data);

    normalFrameBuffer.Unbind();

    glBindTexture(GL_TEXTURE_2D, 0);

    glActiveTexture(GL_TEXTURE0);

    normalProgram->Unbind();

    glPopAttrib();

    glFinish();
}


void Resize::time(GPUTexture * source, Img<unsigned short> & dest)
{
    timeFrameBuffer.Bind();

    glPushAttrib(GL_VIEWPORT_BIT);

    glViewport(0, 0, timeRenderBuffer.width, timeRenderBuffer.height);

    glClearColor(0, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    timeProgram->Bind();

    timeProgram->setUniform(Uniform("eSampler", 0));

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, source->texture->tid);

    glDrawArrays(GL_POINTS, 0, 1);

    glReadPixels(0, 0, timeRenderBuffer.width, timeRenderBuffer.height, GL_LUMINANCE_INTEGER_EXT, GL_UNSIGNED_SHORT, dest.data);

    timeFrameBuffer.Unbind();

    glBindTexture(GL_TEXTURE_2D, 0);

    glActiveTexture(GL_TEXTURE0);

    timeProgram->Unbind();

    glPopAttrib();

    glFinish();
}

void Resize::vertex_noDownSampling(GPUTexture * source, Img<Eigen::Vector4f> & dest)
{
    rawVertexFrameBuffer.Bind();

    glPushAttrib(GL_VIEWPORT_BIT);

    glViewport(0, 0, 640, 480);

    glClearColor(0, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    rawVertexProgram->Bind();

    rawVertexProgram->setUniform(Uniform("eSampler", 0));

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, source->texture->tid);

    glDrawArrays(GL_POINTS, 0, 1);

    glReadPixels(0, 0, 640, 480, GL_RGBA, GL_FLOAT, dest.data);

    rawVertexFrameBuffer.Unbind();

    glBindTexture(GL_TEXTURE_2D, 0);

    glActiveTexture(GL_TEXTURE0);

    rawVertexProgram->Unbind();

    glPopAttrib();

    glFinish();
}
