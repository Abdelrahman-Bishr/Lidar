/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
//#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include <pcl/pcl_config.h>
std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = true;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar * lidar = new Lidar(cars,0);
    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> * pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();
    pcl::PointCloud<pcl::PointXYZ>::Ptr env = lidar->scan();
    std::pair <pcl::PointCloud<pcl::PointXYZ>::Ptr , pcl::PointCloud<pcl::PointXYZ>::Ptr  > planes ; 
    planes = pointProcessor->SegmentPlane(env,100,0.2);
    
    std::vector <pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
    //clusters = pointProcessor->Clustering(planes.second , 2 , 3 , 30);


    renderPointCloud(viewer,planes.second,"obstacles",Color(0,0,0));
//    renderPointCloud(viewer,planes.first,"road",Color(1,1,0));
//    renderRays(viewer, lidar->position ,env);
//    renderPointCloud(viewer,env, "mmmmmmm");

    int i = 0;
    Color c[3] = {Color(1,1,1),Color(1,0,0),Color(0.5,1,0.2)};

    for (std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>::iterator it = clusters.begin() ; 
                it!=clusters.end() ; it++){
         
        renderPointCloud(viewer , (*it) , std::to_string(i), c[i]);
        Box box = pointProcessor->BoundingBox((*it));
        renderBox(viewer,box,i,c[i]);
        i++;
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


void cityBlock (pcl::visualization::PCLVisualizer::Ptr viewer,ProcessPointClouds<pcl::PointXYZI> * pointProcessor,const pcl::PointCloud<pcl::PointXYZI>::Ptr &realCloud) {
    // get region of interest and decrease resolution
    pcl::PointCloud<pcl::PointXYZI>::Ptr regionCloud = pointProcessor->FilterCloud(
        realCloud,0.25 , Eigen::Vector4f (-60,-6.2,-3,1.0) , Eigen::Vector4f (80,6.5,3,1.0)
    );

    // segment cloud into road and obstacles
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr,pcl::PointCloud<pcl::PointXYZI>::Ptr> planes;
    // from scratch implemented segmentation
    planes = pointProcessor->ransacSegment(regionCloud,60,0.25);
    // pcl segmentation
//    planes = pointProcessor->SegmentPlane(regionCloud,100,0.15);
    //dispaly clouds
    renderPointCloud(viewer,planes.first,"road",Color(1,1,1));
//    renderPointCloud(viewer,planes.second,"obstacles",Color(1,0,0));

    // from scratch implemented segmentation
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> obstacles(pointProcessor->Cluster(planes.second,0.28,8,2000));
    // pcl segmentation
//    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> obstacles(pointProcessor->Clustering(planes.second,0.25,10,2000));

    // render clusters
    for (int i=0;i<obstacles.size();i++){
        renderPointCloud(viewer,obstacles[i],std::to_string(i));
        Box box = pointProcessor->BoundingBox(obstacles[i]);
        renderBox(viewer,box,i);
    }
    
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;
    std::cout<<PCL_VERSION<<std::endl;
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
//    simpleHighway(viewer);

    ProcessPointClouds<pcl::PointXYZI> * pointProcessor = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessor->streamPcd("../src/sensors/data/pcd/data_1/");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr realCloud;

    // load single data file
//    pcl::PointCloud<pcl::PointXYZI>::Ptr realCloud = pointProcessor->loadPcd("../src/sensors/data/pcd/data_1/0000000012.pcd");
//    cityBlock(viewer,pointProcessor,realCloud);

    while (!viewer->wasStopped ())
    {
        //clear scene
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // get next scene 
        realCloud = pointProcessor->loadPcd((*streamIterator).string());
        cityBlock(viewer,pointProcessor,realCloud);
        streamIterator++;

        // play again if data is done
        if (streamIterator == stream.end())
            streamIterator = stream.begin();
        //process and display scene
        cityBlock(viewer,pointProcessor,realCloud);
        viewer->spinOnce ();
    } 
}