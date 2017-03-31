#ifndef MYLCCP_H
#define MYLCCP_H

// Stdlib
#include <stdlib.h>
#include <cmath>
#include <limits.h>

#include <boost/format.hpp>


// PCL input/output
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

//PCL other
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/supervoxel_clustering.h>

// The segmentation class this example is for
#include "lccp.hpp"

// VTK
#include <vtkImageReader2Factory.h>
#include <vtkImageReader2.h>
#include <vtkImageData.h>
#include <vtkImageFlip.h>
#include <vtkPolyLine.h>

/// *****  Type Definitions ***** ///

//typedef pcl::PointXYZRGBA PointT;  // The point type used for input
typedef pcl::PointXYZRGB PointT;
typedef pcl::LCCPSegmentation<PointT>::SupervoxelAdjacencyList SuperVoxelAdjacencyList;


class myLccp{

public:
    myLccp(){

        /// Callback and variables

        show_normals = false, normals_changed = false;
        show_adjacency = false;
        show_supervoxels = false;
        show_help = true;


        ///  Default values of parameters before parsing
        // Supervoxel Stuff
        voxel_resolution = 0.01f;//0.0075f;
        seed_resolution = 0.03f;
        color_importance = 0.0f;
        spatial_importance = 1.0f;
        normal_importance = 4.0f;
       use_single_cam_transform = false;
       use_supervoxel_refinement = false;

        // LCCPSegmentation Stuff
        concavity_tolerance_threshold = 10;
        smoothness_threshold = 0.1;
        min_segment_size = 0;
        use_extended_convexity = false;
        use_sanity_criterion = true;

        k_factor = 0;


    }

public:

    /// Callback and variables

    bool show_normals , normals_changed ;
    bool show_adjacency ;
    bool show_supervoxels ;
    bool show_help;
    float normals_scale;


    ///  Default values of parameters before parsing
    // Supervoxel Stuff
    float voxel_resolution ;
    float seed_resolution;
    float color_importance;
    float spatial_importance;
    float normal_importance ;
    bool use_single_cam_transform ;
    bool use_supervoxel_refinement ;

    // LCCPSegmentation Stuff
    float concavity_tolerance_threshold ;
    float smoothness_threshold ;
    uint32_t min_segment_size ;
    bool use_extended_convexity ;
    bool use_sanity_criterion ;

    unsigned int k_factor;

    pcl::visualization::PCLVisualizer::Ptr viewer;



public:
    //const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr
    int mySeg( pcl::PointCloud<pcl::PointXYZRGB>::Ptr ,pcl::PointCloud<pcl::PointXYZL>::Ptr &lccp_labeled_cloud);

};


#endif // MYLCCP_H
