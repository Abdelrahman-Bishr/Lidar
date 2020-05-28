/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"




pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	// TODO: Fill in this function
	int n_points = cloud->points.size();
	// For max iterations 
	int best_line_inliers = 0;
	
	while(maxIterations--){
	// Randomly sample points and fit plane
		std::unordered_set<int> curr_inliers;
		int p1_ind = rand()% n_points;
		int p2_ind = rand()% n_points;
		int p3_ind = rand()% n_points;
		// make sure of 3 different points
		while (p2_ind==p1_ind || p2_ind==p3_ind || p3_ind==p1_ind ){
			p2_ind = (rand()+p2_ind+1)% n_points;
			p3_ind = (rand()+p3_ind+1)% n_points;
		}
		pcl::PointXYZ p1 = cloud->points[p1_ind];
		pcl::PointXYZ p2 = cloud->points[p2_ind];
		pcl::PointXYZ p3 = cloud->points[p3_ind];
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
			if (distance < distanceTol ){
				inliers++;
				curr_inliers.insert(p);
			}
		}
		if (inliers>best_line_inliers){
			best_line_inliers = inliers;
			inliersResult = curr_inliers;
		}

	}

	// Return indicies of inliers from fitted line with most inliers
	

	return inliersResult;

}

std::pair <pcl::PointCloud<pcl::PointXYZ>::Ptr ,pcl::PointCloud<pcl::PointXYZ>::Ptr > ransacSegment(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,int maxIterations,float distanceTol){
	//get segment (road) indices
	std::unordered_set<int> inliers = Ransac(cloud,maxIterations,distanceTol);
	// create segment clouds
	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else{
			cloudOutliers->points.push_back(point);
			}
	}
	
	return std::make_pair(cloudInliers,cloudOutliers);
} 

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function

	std::pair< pcl::PointCloud<pcl::PointXYZ>::Ptr ,pcl::PointCloud<pcl::PointXYZ>::Ptr > segments;
	segments = ransacSegment(cloud,100,0.15);	
	// Render 2D point cloud with inliers and outliers
	if(segments.first->points.size())
	{
		renderPointCloud(viewer,segments.first,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,segments.second,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
