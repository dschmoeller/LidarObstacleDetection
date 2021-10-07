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
	std::unordered_set<int> inliersResultFinal;
	srand(time(NULL));

	// For max iterations 
	for (int i = 0; i < maxIterations; ++i){
		// Randomly sample subset and fit line
		// Create two random numbers between 0 and length of point cloud

		/* // 2D case
		int idPointX = rand() % cloud->size(); 
		int idPointY = rand() % cloud->size(); 

		// Measure distance between every point and fitted line
		// Line parameters (Ax + By + C = 0)
		float A = (*cloud)[idPointX].y - (*cloud)[idPointY].y; 
		float B = (*cloud)[idPointY].x - (*cloud)[idPointX].x; 
		float C = (*cloud)[idPointX].x * (*cloud)[idPointY].y - (*cloud)[idPointY].x * (*cloud)[idPointX].y; 

		for (auto it = cloud->begin(); it < cloud->end(); ++it){
			float x = (*it).x; 
			float y = (*it).y; 
			float distance = abs(A*x + B*y+ C) / sqrt(A*A + B*B); 
			// If distance is smaller than threshold count it as inlier
			if (distance < distanceTol){
				inliersResult.insert((int)(it - cloud->begin())); 
			}
		} */	
		//3D case
		int idPoint1 = rand() % cloud->size(); 
		int idPoint2 = rand() % cloud->size(); 
		int idPoint3 = rand() % cloud->size();

		// Measure distance between every point and fitted line
		// Line parameters (Ax + By + C = 0)
		float x1 = cloud->points[idPoint1].x; 
		float y1 = cloud->points[idPoint1].y; 
		float z1 = cloud->points[idPoint1].z; 
		float x2 = cloud->points[idPoint2].x; 
		float y2 = cloud->points[idPoint2].y; 
		float z2 = cloud->points[idPoint2].z; 
		float x3 = cloud->points[idPoint3].x; 
		float y3 = cloud->points[idPoint3].y; 
		float z3 = cloud->points[idPoint3].z; 
		float A = (y2-y1)*(z3-z1) - (z2-z1)*(y3-y1); 
		float B = (z2-z1)*(x3-x1) - (x2-x1)*(z3-z1); 
		float C = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1); 
		float D = -(A*x1 + B*y1 + C*z1); 
		
		for (auto it = cloud->begin(); it < cloud->end(); ++it){
			float x = (*it).x; 
			float y = (*it).y; 
			float z = (*it).z; 
			float distance = abs(A*x + B*y+ C*z + D) / sqrt(A*A + B*B + C*C); 
			// If distance is smaller than threshold count it as inlier
			if (distance < distanceTol){
				inliersResult.insert((int)(it - cloud->begin())); 
			}
		} 	

		// Return indicies of inliers from fitted line with most inliers
		if (inliersResult.size() > inliersResultFinal.size()){
			inliersResultFinal = inliersResult;  
		}
		// Clear inliers set for next iteration
		inliersResult.clear(); 
	}
	
	return inliersResultFinal;
}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// Run Ransac in order to segment a plane
	std::unordered_set<int> inliers = Ransac(cloud, 1000, 0.5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
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
