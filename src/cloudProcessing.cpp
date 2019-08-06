// PCL lib Functions for processing point clouds 

#include "cloudProcessing.h"


//constructor:
template<typename PointT>
CloudProcessing<PointT>::CloudProcessing() {}


//de-constructor:
template<typename PointT>
CloudProcessing<PointT>::~CloudProcessing() {}

// computing the size of the pointcloud
template<typename PointT>
void CloudProcessing<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    //std::cout << "inside numpoints" << std::endl;
    std::cout << cloud->points.size() << std::endl;
}

// Filtering the pointcloud
template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr CloudProcessing<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Voxel grid point reduction 

    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter (*cloud_filtered);

  // Region based filtering. Cropping out the data outside of the boundaries of the minPoint and maxPoint
    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
     region.setInputCloud (cloud_filtered);
     region.filter (*cloudRegion);

     std::vector<int> indices;
    // Cropping out the lidar point cloud hitting the roof of the car
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5,-1.7,-1,1));
    roof.setMax(Eigen::Vector4f(2.6,1.7,-.4,1));
    roof.setInputCloud (cloudRegion);
    roof.filter (indices);

  
    pcl::PointIndices::Ptr inliers { new pcl::PointIndices};

    for(int point:indices)
        inliers->indices.push_back(point);

    
    pcl::ExtractIndices<PointT> extract;
     extract.setInputCloud (cloudRegion);
     extract.setIndices (inliers);
      extract.setNegative (true);
      extract.filter (*cloudRegion);


    // end of the time
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}

 //  Creates two new point clouds, one cloud with obstacles and other with segmented plane. 
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> CloudProcessing<PointT>::SeparateClouds(std::unordered_set<int>& inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{

  typename pcl::PointCloud<PointT>::Ptr  planeCloud(new pcl::PointCloud<PointT>());
  typename pcl::PointCloud<PointT>::Ptr obstaclePcloud(new pcl::PointCloud<PointT>());

  for(int index = 0; index < cloud->points.size(); index++)
  {
    pcl::PointXYZI point = cloud->points[index];
    if(inliers.count(index))
      planeCloud->points.push_back(point);
    else
      obstaclePcloud->points.push_back(point);
  }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(planeCloud, obstaclePcloud);
    return segResult;
}

template<typename PointT>
float CloudProcessing<PointT>::pointDistanceToPlane(float a,float b, float c, float d, float x, float y, float z){

  float distance = fabs(a*x+b*y+c*z + d)/sqrt(a*a + b*b + c*c);


  return distance;
}

template<typename PointT>
 void CloudProcessing<PointT>::generateTheInlierSet(float a, float b, float c,float d, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,float distanceTol, std::unordered_set<int>& ModelInliersIndices){


  for(int i=0; i<cloud->size(); i++){

    if(ModelInliersIndices.count(i)>0) 
        continue;

      float distToLine = pointDistanceToPlane(a,b,c,d, cloud->points[i].x,cloud->points[i].y,cloud->points[i].z );

      if(distToLine <=distanceTol){

        ModelInliersIndices.insert(i);
      }
  }
}

// For max iterations 
// Randomly sample subset and fit it to a plane
// Measure distance between every point and the plane
// If distance is smaller than threshold count it as inlier
// Return indicies of inliers from fitted line with most inlier
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> CloudProcessing<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{

  auto startTime = std::chrono::steady_clock::now();

  std::unordered_set<int> inliersResult;
  srand(time(NULL));

  iers

  std::vector<std::unordered_set<int>> inlierSetsVector;
  std::vector<std::vector<float>> modelCoeficients;

 

  for(int i=0; i<maxIterations; i++){ 

    // generating a random point index of the data. Using set to avoid choosing the previosly chosen points
    std::unordered_set<int> ModelInliersIndices;
    while(ModelInliersIndices.size() < 3){

      ModelInliersIndices.insert(rand() % cloud->size() + 1);

    }

  auto itr = ModelInliersIndices.begin();

  pcl::PointXYZI point1 =  cloud->points[*itr];
  itr++;
  pcl::PointXYZI point2 = cloud->points[*itr];
  itr++;
  pcl::PointXYZI point3 = cloud->points[*itr];

  // assigning the coordinates of the points

  float x1 = point1.x;
  float y1 = point1.y;
  float z1 = point1.z;
  

  float x2 = point2.x;
  float y2 = point2.y;
  float z2 = point2.y;

  float x3 = point3.x;
  float y3 = point3.y;
  float z3 = point3.z;

  // equation of plane is  ax + bx + cz + d =0

   float a = (y2-y1)*(z3-z1) - (z2-z1)*(y3-y1);
   float b = (z2-z1)*(x3-x1) - (x2-x1)*(z3-z1);
   float c = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);

   float d  = -(a*x1 + b*y1 + c*z1);



  // generating the inliers based on the model and the distance tolerance

   generateTheInlierSet(a,b,c,d,cloud,distanceThreshold,ModelInliersIndices);


   // checking if the computed inlier set is greater than the current inlier set

   if(ModelInliersIndices.size() > inliersResult.size()){

     inliersResult = ModelInliersIndices;
   }
  

   

}

  // After the inlier generation is finished. We choose the best model which is the one with the maximum inlier set

 

  std::cout << "The maximum inliers are " <<inliersResult.size() << " points" << std::endl;
  
  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime-startTime);

  std::cout << "Ransac took " << elapsedTime.count() << " milliseconds " << std::endl;

  // After the inlier set is determined. The cloud is seperated into an a ground plane aka the inliers and the rest which is the pointcloud containing the environment obstacles
   std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliersResult,cloud);
    return segResult;

}


// This function performs the neighbor search for a target points within the tree. Only the point that is within its distance tolerance is processed as a nearby point.
template<typename PointT>
void CloudProcessing<PointT>::proximitySearch(typename pcl::PointCloud<PointT>::Ptr cloud, int i, KdTree* tree, std::vector<int>& cluster, std:: vector<bool>& processedPoints, float distanceTol){


		processedPoints[i] = true;

		cluster.push_back(i);
    std::vector<float> point; 
    point.push_back( cloud->points[i].x);
    point.push_back( cloud->points[i].y);
    point.push_back( cloud->points[i].z);

		std::vector<int> nearbyPoints = tree->search(point,distanceTol);

		for(int index :nearbyPoints) {
			 //std::cout << "inside nearby " << std::endl;
			if( processedPoints[index] == false) {

				proximitySearch(cloud,index, tree,cluster,processedPoints,distanceTol);
			}
			
			
		}




}

// This function performs the clustering of the obstacle pointclouds. It returns a vector of pointcloud clusters.

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> CloudProcessing<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree,float clusterTolerance, int minSize, int maxSize)
{
  
  // This vector saves the indices of each cloud cluster
  std::vector<std::vector<int>> clusters;

  // This vector saves the pointclouds data of each cloud cluster
  std::vector<typename pcl::PointCloud<PointT>::Ptr> clustersVector;

  // helps for not repeating the same points for different clustering searchs
	std::vector<bool> processedPoints (cloud->size(), false);

	for(int i=0; i<cloud->size(); i++){
		if(processedPoints[i] == false){

			std::vector<int> cluster;

			proximitySearch(cloud,i,tree,cluster,processedPoints,clusterTolerance);

			
      
      if (clusters.size() > minSize || clusters.size() <= maxSize)
      {
        clusters.push_back(cluster);
      }

		}

	}

  std:: cout << "The size of the clusters " << clusters.size() << std:: endl;


  // creating the point clouds

  for (std::vector<std::vector<int>>::const_iterator it = clusters.begin (); it != clusters.end (); ++it)
  {
    typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
    for (std::vector<int>::const_iterator pit = it->begin (); pit != it->end (); ++pit)
      cloud_cluster->points.push_back (cloud->points[*pit]); 

    clustersVector.push_back(cloud_cluster);
  }

	
	return clustersVector;

}

// This function creates bounding boxing

template<typename PointT>
Box CloudProcessing<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}

// This functions saves the pcd data

template<typename PointT>
void CloudProcessing<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}

//This function loads the pcd data

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr CloudProcessing<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}

// This function streams the paths of the each pointcloud
template<typename PointT>
std::vector<boost::filesystem::path> CloudProcessing<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}