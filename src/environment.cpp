/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "render/render.h"
#include "cloudProcessing.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "cloudProcessing.cpp"



void streetScenes(pcl::visualization::PCLVisualizer::Ptr& viewer,CloudProcessing<pcl::PointXYZI>* processCloud2, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud){

    // Loading the pcd data

    //CloudProcessing<pcl::PointXYZI>* processCloud2 = new CloudProcessing<pcl::PointXYZI>();

    //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = processCloud2->loadPcd("../src/data/pcd/data_1/0000000000.pcd");

   // renderPointCloud(viewer,inputCloud,"inputCloud");
    
    // Filtering the data

    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredPcloud = processCloud2->FilterCloud(inputCloud,.01,Eigen::Vector4f(-10,-5,-10,1), Eigen::Vector4f(50,7,10,1));
    //renderPointCloud(viewer,filteredPcloud,"filterCloud");

    
    // Segmenting the data
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentRealCloud = processCloud2->SegmentPlane(filteredPcloud,1000,0.2);
    
    // Rendering the two segmented point clouds
   // renderPointCloud(viewer,segmentRealCloud.first,"planeCloud",Color(1,0,0));
    //renderPointCloud(viewer,segmentRealCloud.second,"obstacleCloud",Color(0,1,0));
   
    // clustering the obstacle pointcloud
     pcl::PointCloud<pcl::PointXYZI>::Ptr obstacleCloud = segmentRealCloud.second;
     //renderPointCloud(viewer,obstacleCloud,"obstacleCloud",Color(0,1,0));

   
    
     std:: cout << "The size of the obstacle cloud is " << obstacleCloud->size() << std::endl;

    
     
    // creating the KD tree
     KdTree* tree = new KdTree;
     //obstacleCloud->size()
     for(int i=0; i<obstacleCloud->size() ; i++){

         std::vector<float> point; 
         point.push_back( obstacleCloud->points[i].x);
         point.push_back( obstacleCloud->points[i].y);
         point.push_back( obstacleCloud->points[i].z);

         //std:: cout << point[0] << " " << point[1] << " " << point[2] << std::endl;


        tree->insert(point,i); 
         
     }



    // The first taks is building the treee

 
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = processCloud2->Clustering(segmentRealCloud.second,tree, .4, 100, 10000);
    
    //std:: cout << "The size of the obstacle cloud clusters is " << cloudClusters.size() << std::endl;
    
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
     //if(cluster->size() > 5){
         processCloud2->numPoints(cluster);
      renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
      Box box =  processCloud2->BoundingBox(cluster);
      renderBox(viewer,box,clusterId);

      ++clusterId;
     //}
      
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
    //simpleHighway(viewer);
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud;

    CloudProcessing<pcl::PointXYZI>* pointProcessorI = new CloudProcessing<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
    
    


    while (!viewer->wasStopped ())
    {
        
        // clear viewer 
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        streetScenes(viewer,pointProcessorI, inputCloudI);
        //cityBlock(viewer, );
        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();
        
        viewer->spinOnce ();
    } 
}
