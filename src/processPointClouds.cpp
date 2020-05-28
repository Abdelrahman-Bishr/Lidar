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

    //reduce point cloud resolution
    pcl::VoxelGrid<PointT> filter;
    filter.setInputCloud(cloud);
    filter.setLeafSize(filterRes,filterRes,filterRes);
    filter.filter(*filteredCloud);

    //get only region of interest (we don't care about far objects)
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(filteredCloud);
    region.filter(*cloudRegion);

    //eliminate car's roof points , they are static an irrelevant
    region.setNegative(true);
    region.setMin(Eigen::Vector4f(-2,-2,-1,1));
    region.setMax(Eigen::Vector4f(2,2,1,1));
    region.setInputCloud(cloudRegion);
    region.filter(*cloudRegion);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

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


template <typename PointT>
void ProcessPointClouds<PointT>::proximity(std::vector<int> & cluster,const std::vector<std::vector<float> > &points,KdTree * tree , bool * visited, float distanceTol , int ps){
	//mark point as visited
	visited[ps] = true;
	cluster.push_back(ps);
	//find points withing distance to the given point to create a cluster
	std::vector<int> nearby = tree->search(points[ps],distanceTol);

	for (int i=0;i<nearby.size();i++){
		//for all cluster points , if a points isn't yet visited , 
		//visit and find points close to it (get their cluster)
		if (!visited[nearby[i]])
			proximity(cluster,points,tree,visited,distanceTol,nearby[i]);
		
	}
}


template <typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol,int minSize , int maxSize)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;
/********************************************************************************/
	int ps = points.size();
	bool * visited = new bool[ps];
	// nothing processed yet
	for (int i =0; i <ps;i++)
		visited[i]=false;
	//check for all points	
	for (int i=0;i<ps;i++){
		if(visited[i])
			continue;
		// if point isn't yet visited , get its cluster into clusters
		std::vector<int> cluster;
		proximity(cluster,points,tree,visited,distanceTol,i);
		if (cluster.size()>= minSize && cluster.size()<=maxSize)
			clusters.push_back(cluster);
	}


/********************************************************************************/	
	return clusters;
}


template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Cluster(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize){
/*********************************************************************************/

    KdTree * tree=new KdTree;
    std::vector<std::vector<float> > points(cloud->points.size());
    for (int i=0;i<cloud->points.size();i++){
        points[i].push_back(cloud->points[i].x);
        points[i].push_back(cloud->points[i].y);
        points[i].push_back(cloud->points[i].z);
        tree->insert(points[i],i);
    }
    std::vector<std::vector<int> > clustersIndices (euclideanCluster(points, tree, clusterTolerance,minSize,maxSize));


/*********************************************************************************/
	// vectorize clusters
  	int clusterId = 0;
	std::vector<typename pcl::PointCloud<PointT>::Ptr> clustersClouds;
  	for(std::vector<int> cluster : clustersIndices)
  	{
  		typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>());
  		for(int indice: cluster)
  			clusterCloud->points.push_back(cloud->points[indice]);
		clustersClouds.push_back(clusterCloud);
		++clusterId;
  	}
	return clustersClouds;
}



template <typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	// TODO: Fill in this function
	int n_points = cloud->points.size();
	// For max iterations 
	int best_inliers = 0;
	
	while(maxIterations--){
	// Randomly sample subset and fit line
		std::unordered_set<int> curr_inliers;
		int p1_ind = rand()% n_points;
		int p2_ind = rand()% n_points;
		int p3_ind = rand()% n_points;
		while (p2_ind==p1_ind || p2_ind==p3_ind || p3_ind==p1_ind ){
			p2_ind = (rand()+p2_ind+1)% n_points;
			p3_ind = (rand()+p3_ind+1)% n_points;
		}
		PointT p1 = cloud->points[p1_ind];
		PointT p2 = cloud->points[p2_ind];
		PointT p3 = cloud->points[p3_ind];
		Vect3 v2(p2.x-p1.x,p2.y-p1.y,p2.z-p1.z);
		Vect3 v3(p3.x-p1.x,p3.y-p1.y,p3.z-p1.z);
		Vect3 vn (v2.y*v3.z - v2.z*v3.y , v2.z*v3.x - v2.x * v3.z , v2.x * v3.y - v2.y * v3.x);

		curr_inliers.insert(p1_ind);
		curr_inliers.insert(p2_ind);
		curr_inliers.insert(p3_ind);
		double D = -1 * (vn.x * p1.x + vn.y * p1.y + vn.z * p1.z);  
		double sq = sqrt(vn.x*vn.x + vn.y*vn.y + vn.z*vn.z);

		int inliers = 0;
	// Measure distance between every point and fitted line
		for (int p =0 ; p<n_points;p++){
	// If distance is smaller than threshold count it as inlier
			double distance = abs(vn.x * cloud->points[p].x + vn.y * 
									   cloud->points[p].y + vn.z * cloud->points[p].z + D ) / sq;
			if (distance <= distanceTol ){
				inliers++;
				curr_inliers.insert(p);
			}
		}
		if (inliers>best_inliers){
			best_inliers = inliers;
			inliersResult = curr_inliers;
            std::cout<<cloud->points[p1_ind].x<<" "<<cloud->points[p1_ind].y<<" "<<cloud->points[p1_ind].z<<"\n";
            std::cout<<cloud->points[p2_ind].x<<" "<<cloud->points[p2_ind].y<<" "<<cloud->points[p2_ind].z<<"\n";
            std::cout<<cloud->points[p3_ind].x<<" "<<cloud->points[p3_ind].y<<" "<<cloud->points[p3_ind].z<<"\n\n";
		}

	}

	// Return indicies of inliers from fitted line with most inliers
	

	return inliersResult;

}


template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr ,typename pcl::PointCloud<PointT>::Ptr > ProcessPointClouds<PointT>::ransacSegment (typename pcl::PointCloud<PointT>::Ptr cloud , int maxIterations,float distanceTol){
	std::unordered_set<int> inliers = Ransac(cloud,maxIterations,distanceTol);
	// create segment clouds
	typename pcl::PointCloud<PointT>::Ptr cloudInliers(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		if(inliers.count(index)){
			cloudInliers->points.push_back(cloud->points[index]);
		}
        else{
			cloudOutliers->points.push_back(cloud->points[index]);
			}
	}
	
	return std::make_pair(cloudInliers,cloudOutliers);
} 
