// PCL lib Functions for processing point clouds 

#ifndef CLOUDPROCESSING_H_
#define CLOUDPROCESSING_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include<unordered_set>
#include <ctime>
#include <chrono>
#include "render/box.h"
#include "kdtree.h"

template<typename PointT>
class CloudProcessing {
public:

    //constructor
    CloudProcessing();
    //deconstructor
    ~CloudProcessing();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);
    float pointDistanceToPlane(float a,float b, float c, float d, float x, float y, float z);
    void generateTheInlierSet(float a, float b, float c,float d, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,float distanceTol, std::unordered_set<int>& ModelInliersIndices);
    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(std::unordered_set<int>&, typename pcl::PointCloud<PointT>::Ptr cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    void proximitySearch(typename pcl::PointCloud<PointT>::Ptr cloud , int i, KdTree* tree, std::vector<int>& cluster, std:: vector<bool>& processedPoints, float distanceTol);
    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, float clusterTolerance, int minSize, int maxSize);

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
  
};
#endif /* CLOUDPROCESSING_H_ */