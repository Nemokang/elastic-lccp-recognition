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

#include "ElasticFusion.h"
#include <string>

ElasticFusion::ElasticFusion(const int timeDelta,
                             const int countThresh,
                             const float errThresh,
                             const float covThresh,
                             const bool closeLoops,
                             const bool iclnuim,
                             const bool reloc,
                             const float photoThresh,
                             const float confidence,
                             const float depthCut,
                             const float icpThresh,
                             const bool fastOdom,
                             const float fernThresh,
                             const bool so3,
                             const bool frameToFrameRGB,
                             const std::string fileName)
    : frameToModel(Resolution::getInstance().width(),
                   Resolution::getInstance().height(),
                   Intrinsics::getInstance().cx(),
                   Intrinsics::getInstance().cy(),
                   Intrinsics::getInstance().fx(),
                   Intrinsics::getInstance().fy()),
      modelToModel(Resolution::getInstance().width(),
                   Resolution::getInstance().height(),
                   Intrinsics::getInstance().cx(),
                   Intrinsics::getInstance().cy(),
                   Intrinsics::getInstance().fx(),
                   Intrinsics::getInstance().fy()),
      ferns(500, depthCut * 1000, photoThresh),
      saveFilename(fileName),
      currPose(Eigen::Matrix4f::Identity()),
      tick(1),
      timeDelta(timeDelta),
      icpCountThresh(countThresh),
      icpErrThresh(errThresh),
      covThresh(covThresh),
      deforms(0),
      fernDeforms(0),
      consSample(20),
      resize(Resolution::getInstance().width(),
             Resolution::getInstance().height(),
             Resolution::getInstance().width() / consSample,
             Resolution::getInstance().height() / consSample),
      imageBuff(Resolution::getInstance().rows() / consSample, Resolution::getInstance().cols() / consSample),
      consBuff(Resolution::getInstance().rows() / consSample, Resolution::getInstance().cols() / consSample),
      normBuff(Resolution::getInstance().rows(), Resolution::getInstance().cols()),//----------------------------------new add
      vertexBuff(Resolution::getInstance().rows(), Resolution::getInstance().cols()),//----------------------------------new add---------------
      colorTimeBuff(Resolution::getInstance().rows(), Resolution::getInstance().cols()),//----------------------------------new add
      vertConfBuff(Resolution::getInstance().rows(), Resolution::getInstance().cols()),//----------------------------------new add
      normalRadBuff(Resolution::getInstance().rows(), Resolution::getInstance().cols()),//----------------------------------new add
      timesBuff(Resolution::getInstance().rows() / consSample, Resolution::getInstance().cols() / consSample),
      closeLoops(closeLoops),
      iclnuim(iclnuim),
      reloc(reloc),
      lost(false),
      lastFrameRecovery(false),
      trackingCount(0),
      maxDepthProcessed(20.0f),
      rgbOnly(false),
      icpWeight(icpThresh),
      pyramid(true),
      fastOdom(fastOdom),
      confidenceThreshold(confidence),
      fernThresh(fernThresh),
      so3(so3),
      frameToFrameRGB(frameToFrameRGB),
      depthCutoff(depthCut),
      find_target(false),
      target_change(false),
      label_count(0)
    // cloud( new pcl::PointCloud<pcl::PointXYZRGB> )//--------new add
{
    createTextures();
    createCompute();
    createFeedbackBuffers();

    std::string filename = fileName;
    filename.append(".freiburg");

    std::ofstream file;
    file.open(filename.c_str(), std::fstream::out);
    file.close();

    Stopwatch::getInstance().setCustomSignature(12431231);
}

ElasticFusion::~ElasticFusion()
{
    if(iclnuim)
    {
        savePly();
    }

    //Output deformed pose graph
    std::string fname = saveFilename;
    fname.append(".freiburg");

    std::ofstream f;
    f.open(fname.c_str(), std::fstream::out);

    for(size_t i = 0; i < poseGraph.size(); i++)
    {
        std::stringstream strs;

        if(iclnuim)
        {
            strs << std::setprecision(6) << std::fixed << (double)poseLogTimes.at(i) << " ";
        }
        else
        {
            strs << std::setprecision(6) << std::fixed << (double)poseLogTimes.at(i) / 1000000.0 << " ";
        }

        Eigen::Vector3f trans = poseGraph.at(i).second.topRightCorner(3, 1);
        Eigen::Matrix3f rot = poseGraph.at(i).second.topLeftCorner(3, 3);

        f << strs.str() << trans(0) << " " << trans(1) << " " << trans(2) << " ";

        Eigen::Quaternionf currentCameraRotation(rot);

        f << currentCameraRotation.x() << " " << currentCameraRotation.y() << " " << currentCameraRotation.z() << " " << currentCameraRotation.w() << "\n";
    }

    f.close();

    for(std::map<std::string, GPUTexture*>::iterator it = textures.begin(); it != textures.end(); ++it)
    {
        delete it->second;
    }

    textures.clear();

    for(std::map<std::string, ComputePack*>::iterator it = computePacks.begin(); it != computePacks.end(); ++it)
    {
        delete it->second;
    }

    computePacks.clear();

    for(std::map<std::string, FeedbackBuffer*>::iterator it = feedbackBuffers.begin(); it != feedbackBuffers.end(); ++it)
    {
        delete it->second;
    }

    feedbackBuffers.clear();
}

void ElasticFusion::createTextures()
{
    textures[GPUTexture::RGB] = new GPUTexture(Resolution::getInstance().width(),
                                               Resolution::getInstance().height(),
                                               GL_RGBA,
                                               GL_RGB,
                                               GL_UNSIGNED_BYTE,
                                               true,
                                               true);

    /***************************************************************************************************/
    textures[GPUTexture::LABEL_RGB] = new GPUTexture(Resolution::getInstance().width(),
                                                     Resolution::getInstance().height(),
                                                     GL_LUMINANCE32F_ARB,
                                                     GL_LUMINANCE,
                                                     GL_FLOAT);
    /***************************************************************************************************/

    textures[GPUTexture::DEPTH_RAW] = new GPUTexture(Resolution::getInstance().width(),
                                                     Resolution::getInstance().height(),
                                                     GL_LUMINANCE16UI_EXT,
                                                     GL_LUMINANCE_INTEGER_EXT,
                                                     GL_UNSIGNED_SHORT);

    textures[GPUTexture::DEPTH_FILTERED] = new GPUTexture(Resolution::getInstance().width(),
                                                          Resolution::getInstance().height(),
                                                          GL_LUMINANCE16UI_EXT,
                                                          GL_LUMINANCE_INTEGER_EXT,
                                                          GL_UNSIGNED_SHORT,
                                                          false,
                                                          true);

    textures[GPUTexture::DEPTH_METRIC] = new GPUTexture(Resolution::getInstance().width(),
                                                        Resolution::getInstance().height(),
                                                        GL_LUMINANCE32F_ARB,
                                                        GL_LUMINANCE,
                                                        GL_FLOAT);

    textures[GPUTexture::DEPTH_METRIC_FILTERED] = new GPUTexture(Resolution::getInstance().width(),
                                                                 Resolution::getInstance().height(),
                                                                 GL_LUMINANCE32F_ARB,
                                                                 GL_LUMINANCE,
                                                                 GL_FLOAT);

    textures[GPUTexture::DEPTH_NORM] = new GPUTexture(Resolution::getInstance().width(),
                                                      Resolution::getInstance().height(),
                                                      GL_LUMINANCE,
                                                      GL_LUMINANCE,
                                                      GL_FLOAT,
                                                      true);
}

void ElasticFusion::createCompute()
{
    computePacks[ComputePack::NORM] = new ComputePack(loadProgramFromFile("empty.vert", "depth_norm.frag", "quad.geom"),
                                                      textures[GPUTexture::DEPTH_NORM]->texture);

    computePacks[ComputePack::FILTER] = new ComputePack(loadProgramFromFile("empty.vert", "depth_bilateral.frag", "quad.geom"),
                                                        textures[GPUTexture::DEPTH_FILTERED]->texture);

    computePacks[ComputePack::METRIC] = new ComputePack(loadProgramFromFile("empty.vert", "depth_metric.frag", "quad.geom"),
                                                        textures[GPUTexture::DEPTH_METRIC]->texture);

    computePacks[ComputePack::METRIC_FILTERED] = new ComputePack(loadProgramFromFile("empty.vert", "depth_metric.frag", "quad.geom"),
                                                                 textures[GPUTexture::DEPTH_METRIC_FILTERED]->texture);
}

void ElasticFusion::createFeedbackBuffers()
{
    feedbackBuffers[FeedbackBuffer::RAW] = new FeedbackBuffer(loadProgramGeomFromFile("vertex_feedback.vert", "vertex_feedback.geom"));
    feedbackBuffers[FeedbackBuffer::FILTERED] = new FeedbackBuffer(loadProgramGeomFromFile("vertex_feedback.vert", "vertex_feedback.geom"));
}

void ElasticFusion::computeFeedbackBuffers()
{
    TICK("feedbackBuffers");
    //得到了顶点，颜色，法向量等
    feedbackBuffers[FeedbackBuffer::RAW]->compute(textures[GPUTexture::RGB]->texture,
            textures[GPUTexture::DEPTH_METRIC]->texture,
            textures[GPUTexture::LABEL_RGB]->texture,//-------------------------new add
            tick,
            maxDepthProcessed);//20

    feedbackBuffers[FeedbackBuffer::FILTERED]->compute(textures[GPUTexture::RGB]->texture,
            textures[GPUTexture::DEPTH_METRIC_FILTERED]->texture,
            textures[GPUTexture::LABEL_RGB]->texture,//-------------------------new add
            tick,
            maxDepthProcessed);
    TOCK("feedbackBuffers");
}

bool ElasticFusion::denseEnough(const Img<Eigen::Matrix<unsigned char, 3, 1>> & img)
{
    int sum = 0;

    for(int i = 0; i < img.rows; i++)
    {
        for(int j = 0; j < img.cols; j++)
        {
            sum += img.at<Eigen::Matrix<unsigned char, 3, 1>>(i, j)(0) > 0 &&
                                                             img.at<Eigen::Matrix<unsigned char, 3, 1>>(i, j)(1) > 0 &&
                                                                                                       img.at<Eigen::Matrix<unsigned char, 3, 1>>(i, j)(2) > 0;
        }
    }

    return float(sum) / float(img.rows * img.cols) > 0.75f;
}

void ElasticFusion::processFrame(const unsigned char * rgb,
                                 const unsigned short * depth,
                                 //const unsigned char * labelrgb,
                                 const int64_t & timestamp,
                                 const Eigen::Matrix4f * inPose,
                                 const float weightMultiplier,
                                 const bool bootstrap)
{
    TICK("Run");
    //cv::Mat ma;

    textures[GPUTexture::DEPTH_RAW]->texture->Upload(depth, GL_LUMINANCE_INTEGER_EXT, GL_UNSIGNED_SHORT);
    textures[GPUTexture::RGB]->texture->Upload(rgb, GL_RGB, GL_UNSIGNED_BYTE);

    //textures[GPUTexture::LABEL_RGB]->texture->Upload(labelrgb,  GL_RGB ,GL_UNSIGNED_BYTE);


    /* Preprocess 阶段
        将当前深度图，利用相机内置矩阵Ｋ转化为一个metirc vertex map,
        再将这个map利用 bilateral filter转化为less noise的Ｖt（顶点图）。
        少噪声的Vt再转化为Ｎt(法向图)
    */
    TICK("Preprocess");

    filterDepth();//ComputePack::FILTER
    metriciseDepth();//METRIC----DEPTH_RAW , METRIC_FILTERED-----DEPTH_FILTERED





    TOCK("Preprocess");

    //First run
    if(tick == 1)
    {
        /*********  begin  ************************************************************************************************************/

        segmentation.vertex(textures[GPUTexture::DEPTH_METRIC],currPose,0);//DEPTH_RAW DEPTH_METRIC

        resize.vertex_noDownSampling(&segmentation.vertexTexture, vertexBuff);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud ( new pcl::PointCloud<pcl::PointXYZRGB> );
        int size = 640*480;
        // 遍历深度图
        for (int i = 0; i < 480; i++){
            for (int j=0; j < 640; j++)
            {

                // d 存在值，则向点云增加一个点
                pcl::PointXYZRGB p;

                //Intrinsics & getInstance(float fx = 0,float fy = 0,float cx = 0,float cy = 0);
                //Intrinsics::getInstance(558, 558, 315, 241);
                // 计算这个点的空间坐标
                p.x = vertexBuff.at<Eigen::Vector4f>(i, j)(0);
                p.y = vertexBuff.at<Eigen::Vector4f>(i, j)(1);
                p.z =vertexBuff.at<Eigen::Vector4f>(i, j)(2);

                // 从rgb图像中获取它的颜色
                // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
                int index=(640*i+j)*3;
                p.b = rgb[index+2];
                p.g = rgb[index+1];
                p.r = rgb[index];

                // 把p加入到点云中
                cloud->points.push_back( p );
            }
        }
        // 设置并保存点云
        cloud->height = 1;
        cloud->width = cloud->points.size();



        pcl::PointCloud<pcl::PointXYZL>::Ptr cloudseg;
        myLccp mylccp;
        mylccp.mySeg(cloud,cloudseg);


        float labelColor[640*480];

        /*for(int i=0;i<size;i++)
        {
            labelColor[i]=cloudseg->points[i].label;
        }*/
        std::map<int,float>lab_map;
        for(int i=0;i<size;i++)
        {
            int temp=cloudseg->points[i].label;
            if(temp==0)
            {
                labelColor[i]=0;
            }
            else
            {
                if(lab_map[temp]==0)
                {
                    label_count++;
                    lab_map[temp]=label_count;
                }
                labelColor[i]=lab_map[temp];
            }
        }
        textures[GPUTexture::LABEL_RGB]->texture->Upload(labelColor,  GL_LUMINANCE, GL_FLOAT );

        computeFeedbackBuffers();//compute中初始化了*feedbackBuffers的数据(着色器内部计算法向量)

        //Eigen::Vector4f * mapData = globalModel.downloadMap();

        //globalModel初始化的数据都来自于*feedbackBuffers，然后将*feedbackBuffers数据赋值到模型点上
        globalModel.initialise(*feedbackBuffers[FeedbackBuffer::RAW], *feedbackBuffers[FeedbackBuffer::FILTERED]);

        frameToModel.initFirstRGB(textures[GPUTexture::RGB]);//用到了cuda
    }
    else
    {//Eigen::Vector4f * mapData = globalModel.downloadMap();

        Eigen::Matrix4f lastPose = currPose;

        bool trackingOk = true;

        if(bootstrap || !inPose)
        {
            TICK("autoFill");
            resize.image(indexMap.imageTex(), imageBuff);// Img<Eigen::Matrix<unsigned char, 3, 1>> imageBuff;
            bool shouldFillIn = !denseEnough(imageBuff);//imageBuff失真率<０.25
            TOCK("autoFill");

            TICK("odomInit");
            //WARNING initICP* must be called before initRGB*
            frameToModel.initICPModel(shouldFillIn ? &fillIn.vertexTexture : indexMap.vertexTex(),
                                      shouldFillIn ? &fillIn.normalTexture : indexMap.normalTex(),
                                      maxDepthProcessed, currPose);
            frameToModel.initRGBModel((shouldFillIn || frameToFrameRGB) ? &fillIn.imageTexture : indexMap.imageTex());

            frameToModel.initICP(textures[GPUTexture::DEPTH_FILTERED], maxDepthProcessed);
            frameToModel.initRGB(textures[GPUTexture::RGB]);
            TOCK("odomInit");

            if(bootstrap)
            {
                assert(inPose);//断言
                currPose = currPose * (*inPose);
            }

            Eigen::Vector3f trans = currPose.topRightCorner(3, 1);//平移矩阵tt
            Eigen::Matrix<float, 3, 3, Eigen::RowMajor> rot = currPose.topLeftCorner(3, 3);//Ｒt相机矩阵

            TICK("odom");
            frameToModel.getIncrementalTransformation(trans,
                                                      rot,
                                                      rgbOnly,//false
                                                      icpWeight,//10
                                                      pyramid,//true
                                                      fastOdom,//false
                                                      so3);//true

            //rgbOnly：只利用 RGB 信息进行跟踪，icpWeight：计算位姿时 icp 点云配准占的比重
            //pyramid：配准使用金字塔模式，fastOdom：使用快速跟踪算法
            TOCK("odom");

            trackingOk = !reloc || frameToModel.lastICPError < 1e-04;

            if(reloc)
            {
                if(!lost)
                {
                    Eigen::MatrixXd covariance = frameToModel.getCovariance();

                    for(int i = 0; i < 6; i++)
                    {
                        if(covariance(i, i) > 1e-04)
                        {
                            trackingOk = false;
                            break;
                        }
                    }

                    if(!trackingOk)
                    {
                        trackingCount++;

                        if(trackingCount > 10)
                        {
                            lost = true;
                        }
                    }
                    else
                    {
                        trackingCount = 0;
                    }
                }
                else if(lastFrameRecovery)
                {
                    Eigen::MatrixXd covariance = frameToModel.getCovariance();

                    for(int i = 0; i < 6; i++)
                    {
                        if(covariance(i, i) > 1e-04)
                        {
                            trackingOk = false;
                            break;
                        }
                    }

                    if(trackingOk)
                    {
                        lost = false;
                        trackingCount = 0;
                    }

                    lastFrameRecovery = false;
                }
            }

            currPose.topRightCorner(3, 1) = trans;
            currPose.topLeftCorner(3, 3) = rot;
        }
        else
        {
            currPose = *inPose;
        }

        Eigen::Matrix4f diff = currPose.inverse() * lastPose;//inverse逆

        Eigen::Vector3f diffTrans = diff.topRightCorner(3, 1);
        Eigen::Matrix3f diffRot = diff.topLeftCorner(3, 3);

        //Weight by velocity
        float weighting = std::max(diffTrans.norm(), rodrigues2(diffRot).norm());

        float largest = 0.01;
        float minWeight = 0.5;
        if(weighting > largest)
        {
            weighting = largest;
        }

        weighting = std::max(1.0f - (weighting / largest), minWeight) * weightMultiplier;

        std::vector<Ferns::SurfaceConstraint> constraints;

        predict();

        //----------------------------------------------------------global loop closure--------------------------------------------------------

        Eigen::Matrix4f recoveryPose = currPose;

        if(closeLoops)
        {
            lastFrameRecovery = false;

            TICK("Ferns::findFrame");
            recoveryPose = ferns.findFrame(constraints,
                                           currPose,
                                           &fillIn.vertexTexture,
                                           &fillIn.normalTexture,
                                           &fillIn.imageTexture,
                                           tick,
                                           lost);
            TOCK("Ferns::findFrame");
        }

        std::vector<float> rawGraph;

        bool fernAccepted = false;

        if(closeLoops && ferns.lastClosest != -1)
        {
            if(lost)
            {
                currPose = recoveryPose;
                lastFrameRecovery = true;
            }
            else
            {
                for(size_t i = 0; i < constraints.size(); i++)
                {
                    globalDeformation.addConstraint(constraints.at(i).sourcePoint,
                                                    constraints.at(i).targetPoint,
                                                    tick,
                                                    ferns.frames.at(ferns.lastClosest)->srcTime,
                                                    true);
                }

                for(size_t i = 0; i < relativeCons.size(); i++)
                {
                    globalDeformation.addConstraint(relativeCons.at(i));
                }

                //rawGraph 保存了控制点坐标和优化之后的 R和t
                if(globalDeformation.constrain(ferns.frames, rawGraph, tick, true, poseGraph, true))
                {
                    currPose = recoveryPose;

                    poseMatches.push_back(PoseMatch(ferns.lastClosest, ferns.frames.size(), ferns.frames.at(ferns.lastClosest)->pose, currPose, constraints, true));

                    fernDeforms += rawGraph.size() > 0;

                    fernAccepted = true;
                }
            }
        }

        //----------------------------------------------------------local loop closure--------------------------------------------------------

        //If we didn't match to a fern
        if(!lost && closeLoops && rawGraph.size() == 0)
        {
            //Only predict old view, since we just predicted the current view for the ferns (which failed!)
            TICK("IndexMap::INACTIVE");
            indexMap.combinedPredict(currPose,
                                     globalModel.model(),
                                     maxDepthProcessed,
                                     confidenceThreshold,
                                     0,
                                     tick - timeDelta,
                                     timeDelta,
                                     IndexMap::INACTIVE);
            TOCK("IndexMap::INACTIVE");

            //WARNING initICP* must be called before initRGB*
            modelToModel.initICPModel(indexMap.oldVertexTex(), indexMap.oldNormalTex(), maxDepthProcessed, currPose);
            modelToModel.initRGBModel(indexMap.oldImageTex());

            modelToModel.initICP(indexMap.vertexTex(), indexMap.normalTex(), maxDepthProcessed);
            modelToModel.initRGB(indexMap.imageTex());

            Eigen::Vector3f trans = currPose.topRightCorner(3, 1);
            Eigen::Matrix<float, 3, 3, Eigen::RowMajor> rot = currPose.topLeftCorner(3, 3);

            modelToModel.getIncrementalTransformation(trans,
                                                      rot,
                                                      false,
                                                      10,
                                                      pyramid,
                                                      fastOdom,
                                                      false);

            Eigen::MatrixXd covar = modelToModel.getCovariance();
            bool covOk = true;

            for(int i = 0; i < 6; i++)
            {
                if(covar(i, i) > covThresh)
                {
                    covOk = false;
                    break;
                }
            }

            Eigen::Matrix4f estPose = Eigen::Matrix4f::Identity();

            estPose.topRightCorner(3, 1) = trans;
            estPose.topLeftCorner(3, 3) = rot;


            if(covOk && modelToModel.lastICPCount > icpCountThresh && modelToModel.lastICPError < icpErrThresh)
            {
                resize.vertex(indexMap.vertexTex(), consBuff);
                resize.time(indexMap.oldTimeTex(), timesBuff);


                for(int i = 0; i < consBuff.cols; i++)
                {
                    for(int j = 0; j < consBuff.rows; j++)
                    {
                        if(consBuff.at<Eigen::Vector4f>(j, i)(2) > 0 &&
                                consBuff.at<Eigen::Vector4f>(j, i)(2) < maxDepthProcessed &&
                                timesBuff.at<unsigned short>(j, i) > 0)
                        {
                            Eigen::Vector4f worldRawPoint = currPose * Eigen::Vector4f(consBuff.at<Eigen::Vector4f>(j, i)(0),
                                                                                       consBuff.at<Eigen::Vector4f>(j, i)(1),
                                                                                       consBuff.at<Eigen::Vector4f>(j, i)(2),
                                                                                       1.0f);

                            Eigen::Vector4f worldModelPoint = estPose * Eigen::Vector4f(consBuff.at<Eigen::Vector4f>(j, i)(0),
                                                                                        consBuff.at<Eigen::Vector4f>(j, i)(1),
                                                                                        consBuff.at<Eigen::Vector4f>(j, i)(2),
                                                                                        1.0f);

                            constraints.push_back(Ferns::SurfaceConstraint(worldRawPoint, worldModelPoint));

                            localDeformation.addConstraint(worldRawPoint,
                                                           worldModelPoint,
                                                           tick,
                                                           timesBuff.at<unsigned short>(j, i),
                                                           deforms == 0);
                        }
                    }
                }

                std::vector<Deformation::Constraint> newRelativeCons;

                // newRelativeCons 只有在 localDeformation 时才会产生
                if(localDeformation.constrain(ferns.frames, rawGraph, tick, false, poseGraph, false, &newRelativeCons))
                {
                    poseMatches.push_back(PoseMatch(ferns.frames.size() - 1, ferns.frames.size(), estPose, currPose, constraints, false));

                    deforms += rawGraph.size() > 0;

                    currPose = estPose;

                    for(size_t i = 0; i < newRelativeCons.size(); i += newRelativeCons.size() / 3)
                    {
                        relativeCons.push_back(newRelativeCons.at(i));
                    }
                }
            }
        }





        //----------------------------------------------------------fuse new data--------------------------------------------------------




        if(!rgbOnly && trackingOk && !lost)
        {
            TICK("indexMap");
            indexMap.predictIndices(currPose, tick, globalModel.model(), maxDepthProcessed, timeDelta);
            TOCK("indexMap");


            //nemo-add----------
            //读取标准特征用于后续对比
            ifstream in("up.txt",ios::in);
            float standard_feature[8];
            for(int i=0;i<8;i++)
            {
                in>>standard_feature[i];
            }
            in.close();
            //done------------------
            int obj_label=-1;  //识别的对象的label
            std::vector<pcl::PointCloud<pcl::PointXYZLNormal>>label_cloud; //存储点云块
            frameNum++;
            //if(!find_target&&frameNum>50)
            if(frameNum==55)
            {
                //nemo-add---------------
                //读取所有点云,基于label,分为不同的点云块
                std::vector<int>global_label;                                        //存储GM的label
                Eigen::Vector4f * mapdata = globalModel.downloadMap();


                //std::cout<<"666"<<std::endl;
                for(unsigned int i = 0; i < globalModel.lastCount(); i++)
                {
                    pcl::PointXYZLNormal p;
                    Eigen::Vector4f pos = mapdata[(i * 3) + 0];//position
                    Eigen::Vector4f col = mapdata[(i * 3) + 1];//0->color,1->label
                    Eigen::Vector4f nor = mapdata[(i * 3) + 2];//normal

                    p.x=pos[0];    //生成一个PointXYZLNormal类型的点
                    p.y=pos[1];
                    p.z=pos[2];
                    p.label=col[1];
                    p.normal_x=nor[0];
                    p.normal_y=nor[1];
                    p.normal_z=nor[2];

                    bool find_label=false;
                    for(int i=0;i<global_label.size();i++)
                    {
                        if(global_label[i]==col[1])
                        {
                            find_label=true;
                            //if find,add point to label_cloud[i]
                            label_cloud[i].points.push_back(p);
                        }
                    }
                    if(!find_label)
                    {
                        pcl::PointCloud<pcl::PointXYZLNormal> temp_cloud;
                        temp_cloud.points.push_back(p);
                        //temp_cloud.push_back(p);
                        //两个vector数量相同,都含有相同label
                        global_label.push_back(col[1]);        //将label添加到global_label
                        label_cloud.push_back(temp_cloud);     //将点云块放入label_cloud
                    }
                }


                //std::cout<<"global_label数量:"<<global_label.size()<<endl;
                //std::cout<<"label_cloud数量:"<<label_cloud.size()<<endl;

                
                //nemo-add---------------
                //依次遍历所有点云,得到bounding box以及中心
                int cut_times=6;    //n为切割次数
                std::vector<std::vector<float>>cloud_feature;
                std::vector<float>f_angle(8*cut_times-8);         //将角度作为特征

                //获取bounding box相关数据
                for(int i=0;i<label_cloud.size();i++)
                {
                    int pc_count=label_cloud[i].size();    //当前点云的size


                    pcl::PointCloud<pcl::PointXYZLNormal>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZLNormal>(label_cloud[i]));
                    pcl::MomentOfInertiaEstimation <pcl::PointXYZLNormal> feature_extractor;
                    feature_extractor.setInputCloud (ptr_cloud);
                    feature_extractor.compute ();

                    pcl::PointXYZLNormal min_point_OBB;
                    pcl::PointXYZLNormal max_point_OBB;
                    pcl::PointXYZLNormal position_OBB;
                    Eigen::Matrix3f rotational_matrix_OBB;

                    feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);

                    float x_c=position_OBB.x;
                    float y_c=position_OBB.y;
                    float z_c=position_OBB.z;

                    float x_max=x_c+max_point_OBB.x;
                    float y_max=y_c+max_point_OBB.y;
                    float x_min=x_c-max_point_OBB.x;
                    float y_min=y_c-max_point_OBB.y;
                    //点云旋转
                    Eigen::Matrix3f pc_rotate=rotational_matrix_OBB.inverse();  //矩阵求逆
                    for(int j=0;j<pc_count;j++)
                    {
                        float x=label_cloud[i].points[j].x-position_OBB.x;    //旋转坐标=（初始坐标-中心坐标）×旋转矩阵+中心坐标
                        float y=label_cloud[i].points[j].y-position_OBB.y;
                        float z=label_cloud[i].points[j].z-position_OBB.z;
                        float norm_x=label_cloud[i].points[j].normal_x;
                        float norm_y=label_cloud[i].points[j].normal_y;
                        float norm_z=label_cloud[i].points[j].normal_z;

                        label_cloud[i].points[j].x=x*pc_rotate(0,0)+y*pc_rotate(0,1)+z*pc_rotate(0,2)+position_OBB.x;
                        label_cloud[i].points[j].y=x*pc_rotate(1,0)+y*pc_rotate(1,1)+z*pc_rotate(1,2)+position_OBB.y;
                        label_cloud[i].points[j].z=x*pc_rotate(2,0)+y*pc_rotate(2,1)+z*pc_rotate(2,2)+position_OBB.z;

                        //向量旋转可以直接乘以矩阵得到结果

                        label_cloud[i].points[j].normal_x=norm_x*pc_rotate(0,0)+norm_y*pc_rotate(0,1)+norm_z*pc_rotate(0,2);;
                        label_cloud[i].points[j].normal_y=norm_x*pc_rotate(1,0)+norm_y*pc_rotate(1,1)+norm_z*pc_rotate(1,2);;
                        label_cloud[i].points[j].normal_z=norm_x*pc_rotate(2,0)+norm_y*pc_rotate(2,1)+norm_z*pc_rotate(2,2);;

                    }

                    float bb_depth=max_point_OBB.z-min_point_OBB.z;
                    float z_increment=bb_depth/cut_times;
                    float threshold=z_increment*0.025;

                    float z_cur=position_OBB.z-0.5*bb_depth;     //也可以写成position_OBB.z-max_point_OBB.z
                    //旧的bounding box 初始化方法
                    //初始化为第一个点的xyz
                    //                    float x_max=label_cloud[i].points[0].x,x_min=label_cloud[i].points[0].x;
                    //                    float y_max=label_cloud[i].points[0].y,y_min=label_cloud[i].points[0].y;
                    //                    float z_max=label_cloud[i].points[0].z,z_min=label_cloud[i].points[0].z;
                    //                    for(int j=1;j<pc_count;j++)
                    //                    {
                    //                        float x=label_cloud[i].points[j].x;
                    //                        float y=label_cloud[i].points[j].y;
                    //                        float z=label_cloud[i].points[j].z;
                    //                        if(x_max<x)
                    //                        {
                    //                            x_max=x;
                    //                        }
                    //                        if(y_max<y)
                    //                        {
                    //                            y_max=y;
                    //                        }
                    //                        if(z_max<z)
                    //                        {
                    //                            z_max=z;
                    //                        }

                    //                        if(x_min>x)
                    //                        {
                    //                            x_min=x;
                    //                        }
                    //                        if(y_min>y)
                    //                        {
                    //                            y_min=y;
                    //                        }
                    //                        if(z_min>z)
                    //                        {
                    //                            z_min=z;
                    //                        }
                    //                    }
                    

                    //                    //center of bounding box
                    //                    float x_c=(x_max+x_min)/2,y_c=(y_max+y_min)/2,z_c=(z_max+z_min)/2;


                    //                    float z_increment=(z_max-z_min)/cut_times;
                    //                    float z_cur=z_min;



                    //获取五组角度,存放在f_angle中
                    for(int k=1;k<cut_times;k++)
                    {
                        z_cur+=z_increment;
                        //遍历点云,找到z在z_cur附近的点云
                        for(int num=0;num<pc_count;num++)
                        {
                            float z=label_cloud[i].points[num].z;
                            if(z>z_cur-threshold && z<z_cur+threshold)    //满足条件条件的点即当前切割面上的点
                            {
                                float x=label_cloud[i].points[num].x;
                                float y=label_cloud[i].points[num].y;
                                //顺时针 寻找
                                Eigen::Vector3f temp_nor(label_cloud[i].points[num].normal_x,label_cloud[i].points[num].normal_y,label_cloud[i].points[num].normal_z);
                                Eigen::Vector3f location(x_c-x,y_c-y,z_c-z);    //中心位置到该点的向量
                                float cos;
                                cos=acos((temp_nor(0)*location(0)+temp_nor(1)*location(1)+temp_nor(2)*location(2))/sqrt(location(0)*location(0)+location(1)*location(1)+location(2)*location(2)))*180.0/3.141592653;
                                float x1=std::fabs(x-x_c);
                                if(x1<=0.001 && y>y_c)
                                {
                                    //第1个
                                    f_angle[(k-1)*8]=cos;
                                    continue;
                                    //p_normal[(k-1)*8]=temp_nor;
                                }
                                float slope=(y-y_c)/(x-x_c);
                                float slope1=(y_max-y_c)/(x_max-x_c);
                                if(std::fabs(slope-slope1)<0.01)
                                {
                                    //第2个
                                    f_angle[(k-1)*8+1]=cos;
                                    continue;
                                    //p_normal[(k-1)*8+1]=temp_nor;
                                }

                                float y1=std::fabs(y-y_c);
                                if(y1<=0.001 && x>x_c)
                                {
                                    //第3个
                                    f_angle[(k-1)*8+2]=cos;
                                    continue;
                                    //p_normal[(k-1)*8+2]=temp_nor;
                                }
                                float slope2=(y_min-y_c)/(x_max-x_c);
                                if(std::fabs(slope-slope2)<0.01)
                                {
                                    //第4个
                                    f_angle[(k-1)*8+3]=cos;
                                    continue;
                                    //p_normal[(k-1)*8+3]=temp_nor;
                                }
                                if(x1<=0.001 && y<y_c)
                                {
                                    //第5个
                                    f_angle[(k-1)*8+4]=cos;
                                    continue;
                                    //p_normal[(k-1)*8+4]=temp_nor;
                                }
                                float slope3=(y_min-y_c)/(x_min-x_c);
                                if(std::fabs(slope-slope3)<0.01)
                                {
                                    //第6个
                                    f_angle[(k-1)*8+5]=cos;
                                    continue;
                                    //p_normal[(k-1)*8+5]=temp_nor;
                                }
                                if(y1<=0.001 && x<x_c)
                                {
                                    //第7个
                                    f_angle[(k-1)*8+6]=cos;
                                    continue;
                                    //p_normal[(k-1)*8+6]=temp_nor;
                                }
                                float slope4=(y_max-y_c)/(x_min-x_c);
                                if(std::fabs(slope-slope4)<0.01)
                                {
                                    //第8个
                                    f_angle[(k-1)*8+7]=cos;
                                    continue;
                                    //p_normal[(k-1)*8+7]=temp_nor;
                                }
                            }
                        }
                    }
                    cloud_feature.push_back(f_angle);

                }
                //std::cout<<"cloud_feature数量:"<<cloud_feature.size()<<endl;
                std::string plyname;
                std::string txtname;
                std::string path="/home/nemo/Desktop/pc_data/";
                char  temp_file[5];
                int num=1;
                std::vector<int>aa(label_cloud.size(),0);
                for(int i=0;i<label_cloud.size();i++)
                {
                    //std::cout<<"第"<<i<<"块大小:"<<label_cloud.size()<<endl;
                    //输出所有点云数量大于1000的点云快
                    if(label_cloud[i].size()>1000)
                    {
                        sprintf(temp_file,"%04d",num);
                        plyname = temp_file;
                        plyname = path+plyname+".ply";
                        pcl::io::savePLYFile(plyname,label_cloud[i]);
                        txtname = temp_file;
                        txtname = path+txtname+".txt";
                        ofstream out(txtname,ios::out);
                        for(int j=1;j<=cloud_feature[i].size();j++)
                        {
                            out<<cloud_feature[i][j-1];
                            int aa=j%8;
                            if(aa==0)
                            {
                                out<<endl;
                            }
                            else
                            {
                                out<<" ";
                            }
                        }
                        out.close();
                        num++;
                    }
                    //recognition
//                                        if(label_cloud[i].size()>500)
//                                        {
//                                            int count=0;
//                                            for(int j=0;j<cloud_feature[i].size();j++)
//                                            {
//                                                if(cloud_feature[i][j]>standard_feature[j]-10 && cloud_feature[i][j]<standard_feature[j]+10)
//                                                {count++;}
//                                            }

//                                            if(count>=20)
//                                            {
//                                                //识别成功,记录当前i;
//                                                aa[i]=count;

//                                                //find_target=true;
//                                                //pcl::io::savePCDFile("get.pcd",label_cloud[i]);
//                                            }
//                                        }
                                        if(label_cloud[i].size()>500)
                                        {
                                            int count=0;
                                            int temp_count=0;
                                            for(int j=0;j<cloud_feature[i].size();j++)
                                            {
                                                if(cloud_feature[i][j]>standard_feature[j%8]-15 && cloud_feature[i][j]<standard_feature[j%8]+15)
                                                {temp_count++;}
                                                if((j+1)%8==0)
                                                {
                                                    if(temp_count>count)
                                                    {count=temp_count;}
                                                    temp_count=0;
                                                }
                                            }

                                            if(count>=5)
                                            {
                                                //识别成功,记录当前i;
                                                aa[i]=count;

                                                //find_target=true;
                                                //pcl::io::savePCDFile("get.pcd",label_cloud[i]);
                                            }
                                        }
                }
                std::cout<<"识别结束!"<<endl;
                //search max in vector aa
                                int max_count=0;
                                int max_i=-1;
                                for(int i=0;i<aa.size();i++)
                                {
                                    if(aa[i]>max_count)
                                    {
                                        max_count=aa[i];
                                        max_i=i;
                                    }
                                }
                                if(max_i>=0)
                                {
                                    obj_label=global_label[max_i];
                                    cout<<"obj_label:"<<obj_label<<endl;
                                    pcl::io::savePLYFile("get.ply",label_cloud[max_i]);
                                    std::cout<<"识别成功!"<<endl;
                                }
                                else
                                {cout<<"识别失败!"<<endl;}
            }








            std::cout<<"frameNum:"<<frameNum<<std::endl;

            //       ifUpdatelabel=true;
            /*********  begin  ************************************************************************************************************/

            segmentation.vertex(textures[GPUTexture::DEPTH_METRIC],currPose,1);//DEPTH_RAW DEPTH_METRIC DEPTH_METRIC_FILTERED

            resize.vertex_noDownSampling(&segmentation.vertexTexture, vertexBuff);

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud ( new pcl::PointCloud<pcl::PointXYZRGB> );
            int size = 640*480;
            // 遍历深度图
            for (int i = 0; i < 480; i++){
                for (int j=0; j < 640; j++)
                {

                    // d 存在值，则向点云增加一个点
                    pcl::PointXYZRGB p;

                    //Intrinsics & getInstance(float fx = 0,float fy = 0,float cx = 0,float cy = 0);
                    //Intrinsics::getInstance(558, 558, 315, 241);
                    // 计算这个点的空间坐标
                    p.x = vertexBuff.at<Eigen::Vector4f>(i, j)(0);
                    p.y = vertexBuff.at<Eigen::Vector4f>(i, j)(1);
                    p.z = vertexBuff.at<Eigen::Vector4f>(i, j)(2);

                    // 从rgb图像中获取它的颜色
                    // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
                    int index=(640*i+j)*3;
                    p.b = rgb[index+2];
                    p.g = rgb[index+1];
                    p.r = rgb[index];

                    // 把p加入到点云中
                    cloud->points.push_back( p );
                }
            }
            // 设置并保存点云
            cloud->height = 1;
            cloud->width = cloud->points.size();



            pcl::PointCloud<pcl::PointXYZL>::Ptr cloudseg;
            myLccp mylccp;
            mylccp.mySeg(cloud,cloudseg);


            float labelColor[640*480];
            /*for(int i=0;i<size;i++)
                {
                    labelColor[i]=cloudseg->points[i].label;
                }*/
            std::map<int,float>lab_map;
            for(int i=0;i<size;i++)
            {
                int temp=cloudseg->points[i].label;
                if(temp==0)
                {
                    labelColor[i]=0;
                }
                else
                {
                    if(lab_map[temp]==0)
                    {
                        label_count++;
                        lab_map[temp]=label_count;
                    }
                    labelColor[i]=lab_map[temp];
                }
            }

            //-----nemo add--------------
            resize.normal_noDownSampling(indexMap.colorTimeTex(),colorTimeBuff);



            std::vector<int>li;      //投影得到的label
            std::vector<int>li_count;
            std::vector<int>lj;      //当前分割得到的label
            std::vector<int>lj_count;



            for (int i = 0; i < 480; i++)
            {
                for (int j=0; j < 640; j++)
                {

                    int label_pro=colorTimeBuff.at<Eigen::Vector4f>(i, j)(1);
                    bool findi=false;
                    //将新label与vector存储的label进行对比
                    for(int i=0;i<li.size();i++)
                    {
                        if (label_pro==li[i])
                        {
                            li_count[i]++;  //相同则label对应的count加一
                            findi=true;
                            break;
                        }

                    }
                    if(!findi)
                    {
                        li.push_back(label_pro);
                        li_count.push_back(1);
                    }


                    int label_pro1=labelColor[i*640+j];
                    bool findj=false;
                    //将新label与vector存储的label进行对比
                    for(int i=0;i<lj.size();i++)
                    {
                        if (label_pro1==lj[i])
                        {
                            lj_count[i]++;  //相同则label对应的count加一
                            findj=true;
                            break;
                        }

                    }
                    if(!findj)
                    {//std::cout<<"l:"<<label_pro1<<" ";
                        lj.push_back(label_pro1);
                        lj_count.push_back(1);
                    }


                }
            }
            //                           std::cout<<"label i:"<<li.size()<<" "<<li_count[0]<<std::endl;//li
            //                           std::cout<<"label j:"<<lj.size()<<" "<<lj_count[0]<<std::endl;//lj

            //创建二位数组label_ij用于存储(li,lj)
            //vector<vector <int> > ivec(m ,vector<int>(n,0)); //m*n的二维vector，所有元素初始化为0
            std::vector< std::vector<int>> label_ij(li.size(), std::vector<int>(lj.size(),0));

            /*
                            * 运行判据需要的四个数据：
                            *
                            * Img<Eigen::Vector4f> normBuff;
                            * Img<Eigen::Vector4f> vertexBuff;     //当前深度图优化后得到的顶点图 vertexBuff.at<Eigen::Vector4f>(i, j)(0);
                            * Img<Eigen::Vector4f> colorTimeBuff;  //从indexMap.colorTimeTex()
                            * Img<Eigen::Vector4f> vertConfBuff;   //indexMap.vertConfTex()
                            * Img<Eigen::Vector4f> normalRadBuff;  //indexMap.normalRadTex()
                            */
            segmentation.normal(textures[GPUTexture::DEPTH_METRIC_FILTERED],currPose,1);//DEPTH_RAW DEPTH_METRIC DEPTH_METRIC_FILTERED
            resize.normal_noDownSampling(&segmentation.normalTexture,normBuff);
            //resize.normal_noDownSampling(indexMap.vertConfTex(),vertConfBuff);
            resize.normal_noDownSampling(indexMap.normalRadTex(),normalRadBuff);

            //计算label_ij图
            for (int i = 0; i < 480; i++)
            {
                for (int j=0; j < 640; j++)
                {
                    int label_pro = colorTimeBuff.at<Eigen::Vector4f>(i, j)(1);
                    int label_cur = labelColor[i*640+j];

                    //投影深度图
                    float norm_pro[3] = {normalRadBuff.at<Eigen::Vector4f>(i,j)(0),normalRadBuff.at<Eigen::Vector4f>(i,j)(1),normalRadBuff.at<Eigen::Vector4f>(i,j)(2)};
                    //当前深度图
                    float norm_cur[3] = {normBuff.at<Eigen::Vector4f>(i,j)(0),normBuff.at<Eigen::Vector4f>(i,j)(1),normBuff.at<Eigen::Vector4f>(i,j)(2)};

                    //计算对应点的向量的夹角（单位／度）
                    int angle = acos( norm_pro[0]*norm_cur[0] + norm_pro[1]*norm_cur[1] + norm_pro[2]*norm_cur[2] )*180.0/3.141592653;


                    //满足条件，则增加
                    if( angle<20 && angle>-20)
                    {
                        int x =find(li.begin(),li.end(),label_pro) - li.begin();
                        int y =find(lj.begin(),lj.end(),label_cur) - lj.begin();
                        label_ij[x][y]++;
                    }
                }
            }




            //计算label_ij中每列，即lj对应的一列中最大的count值
            std::vector<int> label_ij_maxCount(lj.size());
            //count取最大时，对应的下标
            std::vector<int> label_ij_iIndex(lj.size());
            for(int i=0 ; i < lj.size() ; i++ )
            {
                label_ij_maxCount[i] = 0;
                label_ij_iIndex[i] = 0 ;
                for(int j=0 ; j < li.size() ; j++ )
                {
                    if(label_ij[j][i] > label_ij_maxCount[i])
                    {
                        label_ij_maxCount[i] = label_ij[j][i];
                        label_ij_iIndex[i] = j ;
                    }
                }
            }


            //更新labelColor图
            for(int i=0 ; i < label_ij_maxCount.size() ; i++ )
            {
                if((float)label_ij_maxCount[i]/(float)lj_count[i]>0.3)
                {
                    int li_index = label_ij_iIndex[i];
                    //如果满足条件，则修改labelColor图中的部分点的label值，否则不更新
                    for(int j=0 ; j < size ; j++ )
                    {
                        if((int)labelColor[j] == lj[i])
                        {
                            labelColor[j] = li[ li_index ];
                        }
                    }
                }
            }
            //将obj_label对应的对象颜色改变
            //code
            /*if(find_target && frameNum==50)
                {
                    std::cout<<"修改完成"<<endl;
                    target_change=true;
                    
                    for(int j=0 ; j < size ; j++ )
                    {
                        if((int)labelColor[j] ==obj_label)
                        {
                            labelColor[j] = 4.0;
                            change_count++;
                        }
                    }
                    std::cout<<"change_count:"<<change_count<<endl;
                }*/


            //labelColor修改之后再upload
            textures[GPUTexture::LABEL_RGB]->texture->Upload(labelColor,  GL_LUMINANCE, GL_FLOAT );

            /*********  end  ************************************************************************************************************/









            //            globalModel.fuse(currPose,
            //                             tick,
            //                             textures[GPUTexture::RGB],
            //                             textures[GPUTexture::DEPTH_METRIC],
            //                             textures[GPUTexture::DEPTH_METRIC_FILTERED],
            //                             indexMap.indexTex(),
            //                             indexMap.vertConfTex(),
            //                             indexMap.colorTimeTex(),
            //                             indexMap.normalRadTex(),
            //                             maxDepthProcessed,
            //                             confidenceThreshold,
            //                             weighting);
            globalModel.fuse(currPose,
                             tick,
                             textures[GPUTexture::RGB],
                    textures[GPUTexture::LABEL_RGB],///////////////////////////new add
                    textures[GPUTexture::DEPTH_METRIC],
                    textures[GPUTexture::DEPTH_METRIC_FILTERED],
                    indexMap.indexTex(),
                    indexMap.vertConfTex(),
                    indexMap.colorTimeTex(),
                    indexMap.normalRadTex(),
                    maxDepthProcessed,
                    confidenceThreshold,
                    weighting,
                    ifUpdatelabel);

            ifUpdatelabel=false;



            TICK("indexMap");
            indexMap.predictIndices(currPose, tick, globalModel.model(), maxDepthProcessed, timeDelta);
            TOCK("indexMap");

            //If we're deforming we need to predict the depth again to figure out which
            //points to update the timestamp's of, since a deformation means a second pose update
            //this loop
            if(rawGraph.size() > 0 && !fernAccepted)
            {
                indexMap.synthesizeDepth(currPose,
                                         globalModel.model(),
                                         maxDepthProcessed,
                                         confidenceThreshold,
                                         tick,
                                         tick - timeDelta,
                                         std::numeric_limits<unsigned short>::max());
            }

            globalModel.clean(currPose,
                              tick,
                              indexMap.indexTex(),
                              indexMap.vertConfTex(),
                              indexMap.colorTimeTex(),
                              indexMap.normalRadTex(),
                              indexMap.depthTex(),
                              confidenceThreshold,
                              rawGraph,
                              timeDelta,
                              maxDepthProcessed,
                              fernAccepted);
        }
    }

    poseGraph.push_back(std::pair<unsigned long long int, Eigen::Matrix4f>(tick, currPose));
    poseLogTimes.push_back(timestamp);

    TICK("sampleGraph");

    //在这里初始化 DeformationGraph
    //新获取的点云融合到全局模型中后，每次初始化新的 Deformation Graph，
    //每次初始化新的 Deformation Graph 要比保持更新同一个 graph 计算量小而且还要简单可行。
    //均匀抽取重建好的模型中的点来初始化 Deformation Graph，抽取点的个数和重建好的模型点的个
    //数成正相关，由于每次新的点添加到模型时是按照时间的先后顺序进行的，均与抽取 Deformation Graph 的点也是按照时间先后顺序排列的。
    localDeformation.sampleGraphModel(globalModel.model());

    globalDeformation.sampleGraphFrom(localDeformation);

    TOCK("sampleGraph");

    predict();

    //如果没跟丢则检测是否将当前帧添加为关键帧
    if(!lost)
    {
        processFerns();
        tick++;
    }

    TOCK("Run");
}

void ElasticFusion::processFerns()
{
    TICK("Ferns::addFrame");
    ferns.addFrame(&fillIn.imageTexture, &fillIn.vertexTexture, &fillIn.normalTexture, currPose, tick, fernThresh);
    TOCK("Ferns::addFrame");
}

void ElasticFusion::predict()
{
    TICK("IndexMap::ACTIVE");

    if(lastFrameRecovery)
    {
        indexMap.combinedPredict(currPose,
                                 globalModel.model(),
                                 maxDepthProcessed,
                                 confidenceThreshold,
                                 0,
                                 tick,
                                 timeDelta,
                                 IndexMap::ACTIVE);
    }
    else
    {
        indexMap.combinedPredict(currPose,
                                 globalModel.model(),
                                 maxDepthProcessed,
                                 confidenceThreshold,
                                 tick,
                                 tick,
                                 timeDelta,
                                 IndexMap::ACTIVE);
    }

    //当跟丢的时候，使用当前帧的信息为下一帧配准，passthrough 传递的参数为 lsot
    //当参数没有跟丢时，使用模型投影获取的图像对下一帧图像配准，当模型投影有残缺值时，使用当前帧的图像做补丁-----所以，indexMap是模型投影图像
    TICK("FillIn");
    fillIn.vertex(indexMap.vertexTex(), textures[GPUTexture::DEPTH_FILTERED], lost);
    fillIn.normal(indexMap.normalTex(), textures[GPUTexture::DEPTH_FILTERED], lost);
    fillIn.image(indexMap.imageTex(), textures[GPUTexture::RGB], lost || frameToFrameRGB);
    TOCK("FillIn");

    TOCK("IndexMap::ACTIVE");
}

void ElasticFusion::metriciseDepth()
{
    std::vector<Uniform> uniforms;

    uniforms.push_back(Uniform("maxD", depthCutoff));

    //void compute(pangolin::GlTexture * input, const std::vector<Uniform> * const uniforms = 0);
    computePacks[ComputePack::METRIC]->compute(textures[GPUTexture::DEPTH_RAW]->texture, &uniforms);
    computePacks[ComputePack::METRIC_FILTERED]->compute(textures[GPUTexture::DEPTH_FILTERED]->texture, &uniforms);
}

void ElasticFusion::filterDepth()
{
    std::vector<Uniform> uniforms;

    uniforms.push_back(Uniform("cols", (float)Resolution::getInstance().cols()));
    uniforms.push_back(Uniform("rows", (float)Resolution::getInstance().rows()));
    uniforms.push_back(Uniform("maxD", depthCutoff));

    computePacks[ComputePack::FILTER]->compute(textures[GPUTexture::DEPTH_RAW]->texture, &uniforms);
}

void ElasticFusion::normaliseDepth(const float & minVal, const float & maxVal)
{
    std::vector<Uniform> uniforms;

    uniforms.push_back(Uniform("maxVal", maxVal * 1000.f));
    uniforms.push_back(Uniform("minVal", minVal * 1000.f));

    computePacks[ComputePack::NORM]->compute(textures[GPUTexture::DEPTH_RAW]->texture, &uniforms);
}

void ElasticFusion::savePly()
{
    std::string filename = saveFilename;
    filename.append(".ply");

    // Open file
    std::ofstream fs;
    fs.open (filename.c_str ());

    Eigen::Vector4f * mapData = globalModel.downloadMap();

    int validCount = 0;

    for(unsigned int i = 0; i < globalModel.lastCount(); i++)
    {
        Eigen::Vector4f pos = mapData[(i * 3) + 0];

        if(pos[3] > confidenceThreshold)
        {
            validCount++;
        }
    }

    // Write header
    fs << "ply";
    fs << "\nformat " << "binary_little_endian" << " 1.0";

    // Vertices
    fs << "\nelement vertex "<< validCount;
    fs << "\nproperty float x"
          "\nproperty float y"
          "\nproperty float z";

    fs << "\nproperty uchar red"
          "\nproperty uchar green"
          "\nproperty uchar blue";

    fs << "\nproperty float nx"
          "\nproperty float ny"
          "\nproperty float nz";

    fs << "\nproperty float radius";

    fs << "\nend_header\n";

    // Close the file
    fs.close ();

    // Open file in binary appendable
    std::ofstream fpout (filename.c_str (), std::ios::app | std::ios::binary);

    for(unsigned int i = 0; i < globalModel.lastCount(); i++)
    {
        Eigen::Vector4f pos = mapData[(i * 3) + 0];

        if(pos[3] > confidenceThreshold)
        {
            Eigen::Vector4f col = mapData[(i * 3) + 1];//color
            Eigen::Vector4f nor = mapData[(i * 3) + 2];//normal

            nor[0] *= -1;
            nor[1] *= -1;
            nor[2] *= -1;

            float value;
            memcpy (&value, &pos[0], sizeof (float));
            fpout.write (reinterpret_cast<const char*> (&value), sizeof (float));

            memcpy (&value, &pos[1], sizeof (float));
            fpout.write (reinterpret_cast<const char*> (&value), sizeof (float));

            memcpy (&value, &pos[2], sizeof (float));
            fpout.write (reinterpret_cast<const char*> (&value), sizeof (float));

            unsigned char r = int(col[0]) >> 16 & 0xFF;
            unsigned char g = int(col[0]) >> 8 & 0xFF;
            unsigned char b = int(col[0]) & 0xFF;

            fpout.write (reinterpret_cast<const char*> (&r), sizeof (unsigned char));
            fpout.write (reinterpret_cast<const char*> (&g), sizeof (unsigned char));
            fpout.write (reinterpret_cast<const char*> (&b), sizeof (unsigned char));

            memcpy (&value, &nor[0], sizeof (float));
            fpout.write (reinterpret_cast<const char*> (&value), sizeof (float));

            memcpy (&value, &nor[1], sizeof (float));
            fpout.write (reinterpret_cast<const char*> (&value), sizeof (float));

            memcpy (&value, &nor[2], sizeof (float));
            fpout.write (reinterpret_cast<const char*> (&value), sizeof (float));

            memcpy (&value, &nor[3], sizeof (float));
            fpout.write (reinterpret_cast<const char*> (&value), sizeof (float));
        }
    }

    // Close file
    fs.close ();

    delete [] mapData;
}

Eigen::Vector3f ElasticFusion::rodrigues2(const Eigen::Matrix3f& matrix)
{
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(matrix, Eigen::ComputeFullV | Eigen::ComputeFullU);
    Eigen::Matrix3f R = svd.matrixU() * svd.matrixV().transpose();

    double rx = R(2, 1) - R(1, 2);
    double ry = R(0, 2) - R(2, 0);
    double rz = R(1, 0) - R(0, 1);

    double s = sqrt((rx*rx + ry*ry + rz*rz)*0.25);
    double c = (R.trace() - 1) * 0.5;
    c = c > 1. ? 1. : c < -1. ? -1. : c;

    double theta = acos(c);

    if( s < 1e-5 )
    {
        double t;

        if( c > 0 )
            rx = ry = rz = 0;
        else
        {
            t = (R(0, 0) + 1)*0.5;
            rx = sqrt( std::max(t, 0.0) );
            t = (R(1, 1) + 1)*0.5;
            ry = sqrt( std::max(t, 0.0) ) * (R(0, 1) < 0 ? -1.0 : 1.0);
            t = (R(2, 2) + 1)*0.5;
            rz = sqrt( std::max(t, 0.0) ) * (R(0, 2) < 0 ? -1.0 : 1.0);

            if( fabs(rx) < fabs(ry) && fabs(rx) < fabs(rz) && (R(1, 2) > 0) != (ry*rz > 0) )
                rz = -rz;
            theta /= sqrt(rx*rx + ry*ry + rz*rz);
            rx *= theta;
            ry *= theta;
            rz *= theta;
        }
    }
    else
    {
        double vth = 1/(2*s);
        vth *= theta;
        rx *= vth; ry *= vth; rz *= vth;
    }
    return Eigen::Vector3d(rx, ry, rz).cast<float>();
}

//Sad times ahead
IndexMap & ElasticFusion::getIndexMap()
{
    return indexMap;
}

GlobalModel & ElasticFusion::getGlobalModel()
{
    return globalModel;
}

Ferns & ElasticFusion::getFerns()
{
    return ferns;
}

Deformation & ElasticFusion::getLocalDeformation()
{
    return localDeformation;
}

std::map<std::string, GPUTexture*> & ElasticFusion::getTextures()
{
    return textures;
}

const std::vector<PoseMatch> & ElasticFusion::getPoseMatches()
{
    return poseMatches;
}

const RGBDOdometry & ElasticFusion::getModelToModel()
{
    return modelToModel;
}

const float & ElasticFusion::getConfidenceThreshold()
{
    return confidenceThreshold;
}

void ElasticFusion::setRgbOnly(const bool & val)
{
    rgbOnly = val;
}

void ElasticFusion::setIcpWeight(const float & val)
{
    icpWeight = val;
}

void ElasticFusion::setPyramid(const bool & val)
{
    pyramid = val;
}

void ElasticFusion::setFastOdom(const bool & val)
{
    fastOdom = val;
}

void ElasticFusion::setSo3(const bool & val)
{
    so3 = val;
}

void ElasticFusion::setFrameToFrameRGB(const bool & val)
{
    frameToFrameRGB = val;
}

void ElasticFusion::setConfidenceThreshold(const float & val)
{
    confidenceThreshold = val;
}

void ElasticFusion::setFernThresh(const float & val)
{
    fernThresh = val;
}

void ElasticFusion::setDepthCutoff(const float & val)
{
    depthCutoff = val;
}

const bool & ElasticFusion::getLost() //lel
{
    return lost;
}

const int & ElasticFusion::getTick()
{
    return tick;
}

const int & ElasticFusion::getTimeDelta()
{
    return timeDelta;
}

void ElasticFusion::setTick(const int & val)
{
    tick = val;
}

const float & ElasticFusion::getMaxDepthProcessed()
{
    return maxDepthProcessed;
}

const Eigen::Matrix4f & ElasticFusion::getCurrPose()
{
    return currPose;
}

const int & ElasticFusion::getDeforms()
{
    return deforms;
}

const int & ElasticFusion::getFernDeforms()
{
    return fernDeforms;
}

std::map<std::string, FeedbackBuffer*> & ElasticFusion::getFeedbackBuffers()
{
    return feedbackBuffers;
}
