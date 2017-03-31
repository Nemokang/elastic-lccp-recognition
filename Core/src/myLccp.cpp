#include "myLccp.h"

int myLccp::mySeg( pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr,
                   pcl::PointCloud<pcl::PointXYZL>::Ptr& lccp_labeled_cloud)//---------一定要用引用
{
    normals_scale = seed_resolution / 2.0;
    if (use_extended_convexity)
        k_factor = 1;

    /// -----------------------------------|  Preparations  |-----------------------------------

    /// Create variables needed for preparations
//    pcl::PointCloud<PointT>::Ptr input_cloud_ptr=input_cloud_ptr1;//(new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr input_normals_ptr(new pcl::PointCloud<pcl::Normal>);
    bool has_normals = false;

    /// Preparation of Input: Supervoxel Oversegmentation

    pcl::SupervoxelClustering<PointT> super(voxel_resolution, seed_resolution);
    super.setUseSingleCameraTransform(use_single_cam_transform);
    super.setInputCloud(input_cloud_ptr);
    if (has_normals)
        super.setNormalCloud(input_normals_ptr);
    super.setColorImportance(color_importance);
    super.setSpatialImportance(spatial_importance);
    super.setNormalImportance(normal_importance);
    std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr> supervoxel_clusters;

    //PCL_INFO("Extracting supervoxels\n");
    super.extract(supervoxel_clusters);

    if (use_supervoxel_refinement)
    {
        PCL_INFO("Refining supervoxels\n");
        super.refineSupervoxels(2, supervoxel_clusters);
    }
    //std::stringstream temp;
    //temp << "  Nr. Supervoxels: " << supervoxel_clusters.size() << "\n";
    //PCL_INFO(temp.str().c_str());

    //PCL_INFO("Getting supervoxel adjacency\n");
    std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
    super.getSupervoxelAdjacency(supervoxel_adjacency);

    /// Get the cloud of supervoxel centroid with normals and the colored cloud with supervoxel coloring (this is used for visulization)
    //pcl::PointCloud<pcl::PointNormal>::Ptr sv_centroid_normal_cloud = pcl::SupervoxelClustering<PointT>::makeSupervoxelNormalCloud(supervoxel_clusters);




    /// The Main Step: Perform LCCPSegmentation

    //PCL_INFO("Starting Segmentation\n");
    pcl::LCCPSegmentation<PointT> lccp;

    lccp.setConcavityToleranceThreshold(concavity_tolerance_threshold);

    lccp.setSanityCheck(use_sanity_criterion);

    lccp.setSmoothnessCheck(true, voxel_resolution, seed_resolution, smoothness_threshold);

    //lccp.setKFactor(k_factor);
    lccp.setKFactor(0);
    lccp.setInputSupervoxels(supervoxel_clusters, supervoxel_adjacency);
    //lccp.setMinSegmentSize(min_segment_size);
    lccp.setMinSegmentSize(5);
    lccp.segment();

    //PCL_INFO("Interpolation voxel cloud -> input cloud and relabeling\n");
    pcl::PointCloud<pcl::PointXYZL>::Ptr sv_labeled_cloud = super.getLabeledCloud();
    //pcl::PointCloud<pcl::PointXYZL>::Ptr lccp_labeled_cloud = sv_labeled_cloud->makeShared();
    lccp_labeled_cloud= sv_labeled_cloud->makeShared();
    int aaa=lccp.relabelCloud(*lccp_labeled_cloud);
    return aaa;









}

