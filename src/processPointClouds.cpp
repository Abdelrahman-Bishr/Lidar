// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr filteredCloud(new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr cloudRegion(new pcl::PointCloud<PointT>);


    pcl::VoxelGrid<PointT> filter;
    filter.setInputCloud(cloud);
    filter.setLeafSize(filterRes,filterRes,filterRes);
    filter.filter(*filteredCloud);

    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMin(maxPoint);
    region.setInputCloud(filteredCloud);
    region.filter(*cloudRegion);
    std::cout<<"input cloud size = "<<cloud->points.size()<<std::endl;
    std::cout<<"filtered cloud size = "<<filteredCloud->points.size()<<std::endl;
    std::cout<<"region cloud size = "<<cloudRegion->points.size()<<std::endl;
    // previous filter isn't working as it should , so another option :
    std::vector<int> indices;
    region.filter(indices);
    std::cout<<"indices size = "<<indices.size()<<std::endl;
    if (cloudRegion->points.size() == 0){
        pcl::ExtractIndices<PointT> extract;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        for (int i =0 ; i<indices.size();i++)
            inliers->indices.push_back(indices[i]);
        extract.setInputCloud(filteredCloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloudRegion);
    }
    std::cout<<"region cloud size = "<<cloudRegion->points.size()<<std::endl;




    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

//    return filteredCloud;
    return cloudRegion;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr plane(new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr obstacles(new pcl::PointCloud<PointT> ());
    for (int i=0;i<inliers->indices.size() ; i++){
        plane->points.push_back(cloud->points[inliers->indices[i]]);
    }
    pcl::ExtractIndices<PointT> extractor;
    extractor.setInputCloud(cloud);
    extractor.setIndices(inliers);
    extractor.setNegative(true);
    extractor.filter(*obstacles);


 
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(plane, obstacles);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // TODO:: Fill in this function to find inliers for the cloud.
    /******************************************************************/
    // create segmentation object
    pcl::SACSegmentation<PointT> seg;
    
    // create coefficients
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    
    //optional
    seg.setOptimizeCoefficients(true); 
    //set segmentation parameters , mandatory
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    
    seg.setInputCloud(cloud);
    seg.segment(*inliers,*coefficients);

    if (inliers->indices.size() == 0 ){
        PCL_ERROR ("Could not estimate a planar model for the given datapoints .");
    }



    /**************************************************************/
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
/*************************************************************************************************/
    // create kd tree
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);

    // create clustering object
    pcl::EuclideanClusterExtraction<PointT> ec;

    std::vector< pcl::PointIndices > clusterIndices;
    
    //set clustering object parameters
    ec.setMaxClusterSize(maxSize);
    ec.setMinClusterSize(minSize);
    ec.setClusterTolerance(clusterTolerance);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    // perform extraction
    ec.extract(clusterIndices);

    // retrieve different clusters
    for (std::vector<pcl::PointIndices>::iterator it = clusterIndices.begin() ; it!= clusterIndices.end() ; it++){
        typename pcl::PointCloud<PointT>::Ptr cluster_cloud(new pcl::PointCloud<PointT>);
        for (int i =0 ; i< it->indices.size() ; i++){
            cluster_cloud->points.push_back( cloud->points[ it->indices[i] ]);
        }
        cluster_cloud->width = cluster_cloud->points.size();
        cluster_cloud->height = 1;
        cluster_cloud->is_dense = true;
        clusters.push_back(cluster_cloud);
    }



/*************************************************************************************************/
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
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


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}