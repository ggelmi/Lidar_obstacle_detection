/* \author Guled Elmi */
// Lidar perception pipeline

#include "render/render.h"
#include "cloudProcessing.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "cloudProcessing.cpp"


// This function puts together the core functionalites of the lidar perception pipeline
void streetScenes(pcl::visualization::PCLVisualizer::Ptr& viewer,CloudProcessing<pcl::PointXYZI>* processCloud2, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud){

    
    
    // The filtering function reduces the computational complexity of the algorithm by reducing the size of the pointcloud to a manageable size

    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredPcloud = processCloud2->FilterCloud(inputCloud,.01,Eigen::Vector4f(-10,-5,-10,1), Eigen::Vector4f(50,7,10,1));

    // Visualizing the filtered point cloud

    //renderPointCloud(viewer,filteredPcloud,"filterCloud");

    
    // Segmenting the pointcloud data into two parts. Part1 is the ground plane point cloud and part 2 is the obstacles point cloud.
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentRealCloud = processCloud2->SegmentPlane(filteredPcloud,1000,0.2);
    
    // Rendering the two segmented point clouds
   // renderPointCloud(viewer,segmentRealCloud.first,"planeCloud",Color(1,0,0));
    //renderPointCloud(viewer,segmentRealCloud.second,"obstacleCloud",Color(0,1,0));
   
    // This line singles out the obstacles point cloud to perform clustering
     pcl::PointCloud<pcl::PointXYZI>::Ptr obstacleCloud = segmentRealCloud.second;
    
     std:: cout << "The size of the obstacle cloud is " << obstacleCloud->size() << std::endl;

    // The first step of the clustering algorithm is to create the K-d tree data structure which produces faster clustering and search compared to plain nearest neighbors distance clustering
     
    // creating the KD tree
     KdTree* tree = new KdTree;
     
     // inserting the obstacle point cloud points into the tree structure
     for(int i=0; i<obstacleCloud->size() ; i++){

         std::vector<float> point; 
         point.push_back( obstacleCloud->points[i].x);
         point.push_back( obstacleCloud->points[i].y);
         point.push_back( obstacleCloud->points[i].z);
        tree->insert(point,i); 
         
     }



    // After the tree is built, the second task is to perform clustering
 
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = processCloud2->Clustering(segmentRealCloud.second,tree, .4, 100, 10000);
    
    //std:: cout << "The size of the obstacle cloud clusters is " << cloudClusters.size() << std::endl;
    
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
    // This piece goes through the clusters and renders them into the viewer. Then each cluster is being covered with a bounded box
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
    
         processCloud2->numPoints(cluster);
      renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
      Box box =  processCloud2->BoundingBox(cluster);
      renderBox(viewer,box,clusterId);

      ++clusterId;
     
      
    }
    
    
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    
    
    

    // creating an instance object of the cloudProcessing class
    CloudProcessing<pcl::PointXYZI>* pointProcessorI = new CloudProcessing<pcl::PointXYZI>();
    // streaming the pointclouds and saving their paths into a vector
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/data/pcd/data_1");
    auto streamIterator = stream.begin();
    // creating the pointer to hold the input cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
    
    

    
    while (!viewer->wasStopped ())
    {
        
        // clear viewer 
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        
        // getting a pointcloud
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        // performing lidar perception module
        streetScenes(viewer,pointProcessorI, inputCloudI);
        // updating the iterator
        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();
        
        viewer->spinOnce ();
    } 
}
