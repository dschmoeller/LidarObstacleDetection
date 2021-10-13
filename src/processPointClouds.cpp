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
    // Apply voxel filter
    typename pcl::PointCloud<PointT>::Ptr voxelCloud (new pcl::PointCloud<PointT>()); 
    pcl::VoxelGrid<PointT> voxelFilter; 
    voxelFilter.setInputCloud(cloud); 
    voxelFilter.setLeafSize(filterRes, filterRes, filterRes); 
    voxelFilter.filter(*voxelCloud); 
    // Apply crop box for region of interest
    typename pcl::PointCloud<PointT>::Ptr ROICloud (new pcl::PointCloud<PointT>()); 
    pcl::CropBox<PointT> cropBox; 
    cropBox.setMin(minPoint); 
    cropBox.setMax(maxPoint); 
    cropBox.setInputCloud(voxelCloud);
    cropBox.filter(*ROICloud);  

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return ROICloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>()); 
    typename pcl::PointCloud<PointT>::Ptr obstacleCloud(new pcl::PointCloud<PointT>());

    // Generate extractor
    pcl::ExtractIndices<PointT> extract; 

    // Generate plane cloud from inliers
    extract.setInputCloud(cloud); 
    extract.setIndices(inliers); 
    extract.setNegative(false); 
    extract.filter(*planeCloud); 

    // Generate object cloud from inliers
    extract.setNegative(true); 
    extract.filter(*obstacleCloud); 

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(planeCloud, obstacleCloud);
    return segResult;
}


template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol){
    
    std::unordered_set<int> inliersResult;
	std::unordered_set<int> inliersResultFinal;
	srand(time(NULL));

	// For max iterations 
	for (int i = 0; i < maxIterations; ++i){
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


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	
    /*
    // Utilize PCL library for segmentation 
    // Create segmentaton object
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients()); 
    pcl::SACSegmentation<PointT> seg; 
    seg.setOptimizeCoefficients(true); 
    seg.setModelType(pcl::SACMODEL_PLANE); 
    seg.setMethodType(pcl::SAC_RANSAC); 
    seg.setMaxIterations(maxIterations); 
    seg.setDistanceThreshold(distanceThreshold); 

    // Segment the largest planar component from the cloud
    seg.setInputCloud(cloud); 
    seg.segment(*inliers, *coefficients); 
    if (inliers->indices.size() == 0){
        std::cerr << "Could not estimate a planar model for the geiven dataset" << std::endl; 
    } 
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    */
    
    // Using RANSAC implementaton from quiz
    // Run Ransac in order to segment a plane
	std::unordered_set<int> inliers = Ransac(cloud, 10, 0.5);
	typename pcl::PointCloud<PointT>::Ptr  cloudInliers(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());
	for(int index = 0; index < cloud->points.size(); index++)
	{
		auto point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult (cloudInliers, cloudOutliers); 

    // Measure time
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    // Return results
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    /*
    // PCL library functions to perform euclidean clustering to group detected obstacles
    // Create KdTree object for fast search
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>); 
    tree->setInputCloud(cloud); 

    // Apply eucledian clustering to extract point clusters 
    std::vector<pcl::PointIndices> clusterIndices; 
    pcl::EuclideanClusterExtraction<PointT> ec; 
    ec.setClusterTolerance(clusterTolerance); 
    ec.setMinClusterSize(minSize); 
    ec.setMaxClusterSize(maxSize); 
    ec.setSearchMethod(tree); 
    ec.setInputCloud(cloud); 
    ec.extract(clusterIndices);

    // Iterate over cluster indices and write corresonding points back to clusters
    for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin(); it != clusterIndices.end(); ++it){
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>); 
        for (const auto& idx : it->indices){
            cloudCluster->push_back((*cloud)[idx]); 
        }
        cloudCluster->width = cloudCluster->size(); 
        cloudCluster->height = 1; 
        cloudCluster->is_dense = true; 

        // Store cluster in vector
        clusters.push_back(cloudCluster); 
    }*/

    // Using kd-tree and eucledian clustering implementation from quiz
    // Implement and fill kd-tree
    KdTree* tree = new KdTree;
    std::vector<std::vector<float>> pointVector; 
    for (int i = 0; i < cloud->size(); i++){
        std::vector<float> point; 
        point.push_back(cloud->points[i].x);  
        point.push_back(cloud->points[i].y);
        //point.push_back(cloud->points[i].z);
        tree->insert(point, i); 
        pointVector.push_back(point); 
    }
    // Apply eucledian clustering algorithm returns clusters of indecies
    std::vector<std::vector<int>> clusterIndecies = euclideanCluster(pointVector, tree, 0.3);
    
    // Iterate over Indecies and build actual point cloud clusters
    for(std::vector<int> singleCluster : clusterIndecies)
  	{
  		typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>());
  		for(int indice : singleCluster)
        {
            PointT p; 
            p.x = cloud->points[indice].x;  
            p.y = cloud->points[indice].y; 
            p.z = cloud->points[indice].z; 
            clusterCloud->points.push_back(p); 
        }
        clusters.push_back(clusterCloud); 
  	}

    // Measure time
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


template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{
    std::vector<std::vector<int>> clusters;
	std::vector<bool> processed(points.size(), true); 
	for(auto it = points.begin(); it != points.end(); ++it){ 
		int idx = std::distance(points.begin(), it); 
		if (processed[idx] == true){
			//Point has not been processed --> Create the cluster 
			std::vector<int> cluster;
			//Find all points within this cluster
			helpCluster(points, tree, distanceTol, processed, idx, cluster); 
			//Add cluster to set of clusters
			clusters.push_back(cluster);  
		}
	}
	return clusters;
}


template<typename PointT>
void ProcessPointClouds<PointT>::helpCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol, std::vector<bool>& processed, int processedIdx, std::vector<int>& cluster){
	// Mark point as processed
	processed[processedIdx] = false; 
	// Add point id to cluster
	cluster.push_back(processedIdx);
	// Get all nearby points
	std::vector<int> nearby = tree->search(points[processedIdx],distanceTol); 
	for(int pId : nearby){
		if (processed[pId] == true){
			// Point has not been processed yet
			helpCluster(points, tree, distanceTol, processed, pId, cluster); 
		}
	}

}


