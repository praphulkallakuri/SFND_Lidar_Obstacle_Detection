// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"

template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(const typename pcl::PointCloud<PointT>::Ptr& cloud) {
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(const typename pcl::PointCloud<PointT>::Ptr& cloud,
        float filterRes, const Eigen::Vector4f& minPoint, const Eigen::Vector4f& maxPoint) {
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    /*** Voxel grid point reduction and region based filtering ***/

    typename pcl::PointCloud<PointT>::Ptr filtered_cloud(new pcl::PointCloud<PointT>());
    // Create the filtering object: down-sample the dataset using a given leaf size
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*filtered_cloud);

    // Filter point cloud that is out of region of interest
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(filtered_cloud);
    region.filter(*filtered_cloud);

    std::vector<int> indices;
    // Filter point cloud on the roof of host vehicle
    region.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    region.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    region.setInputCloud(filtered_cloud);
    region.filter(indices);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    for (int index : indices) {
        inliers->indices.push_back(index);
    }

    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;
    // Extract the point cloud on roof
    extract.setInputCloud(filtered_cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*filtered_cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return filtered_cloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(
        const pcl::PointIndices::Ptr& inliers, const typename pcl::PointCloud<PointT>::Ptr& cloud) {
  // Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstacle_cloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr plane_cloud(new pcl::PointCloud<PointT>());

    // Copy inliers point cloud as plane
    for (int index : inliers->indices) {
        plane_cloud->points.push_back(cloud->points[index]);
    }

    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;
    // Extract the inliers so that we can get obstacles point cloud
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstacle_cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacle_cloud, plane_cloud);
    return segResult;
}


template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::RansacPlane(const typename pcl::PointCloud<PointT>::Ptr& cloud, int maxIterations, float distanceTol) {
    std::unordered_set<int> inliersResult;

    // Will be used to obtain a seed for the random number engine
    std::random_device rd;
    //Standard mersenne_twister_engine seeded with rd()
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0 , cloud->points.size());

    // For max iterations
    for (int i = 0; i < maxIterations; i++) {
        // Randomly sample subset and fit line
        std::unordered_set<int> inliers;

        constexpr int kDimension = 3;
        float x[kDimension], y[kDimension], z[kDimension];
        int j = 0;
        // avoid picking the same point twice
        while(inliers.size() < kDimension) {
            auto index = dis(gen);
            x[j] = cloud->points[index].x;
            y[j] = cloud->points[index].y;
            z[j] = cloud->points[index].z;

            if (0 == inliers.count(index)) {
                inliers.insert(index);
                j++;
            }
        }

        // Two vectors on the plane
        float v0[3] = {x[1] - x[0], y[1] - y[0], z[1] - z[0]};
        float v1[3] = {x[2] - x[0], y[2] - y[0], z[2] - z[0]};
        // Cross product v0 x v1
        float a = v0[1] * v1[2] - v0[2] * v1[1];
        float b = v0[2] * v1[0] - v0[0] * v1[2];
        float c = v0[0] * v1[1] - v0[1] * v1[0];
        float d = -(a * x[0] + b * y[0] + c * z[0]);
        // A constant number that will be used multiple times in following calculation
        auto d2 = sqrt(a*a + b*b + c*c);

        // Measure distance between every point and fitted line
        for (int index = 0; index < cloud->points.size(); index++) {
            if (inliers.count(index) > 0) {
                continue;
            }

            auto point = cloud->points[index];
            float x0 = point.x;
            float y0 = point.y;
            float z0 = point.z;

            float distance = fabs(a*x0 + b*y0 + c*z0 + d) / d2;
            // If distance is smaller than threshold count it as inlier
            if (distance <= distanceTol) {
                inliers.insert(index);
            }
        }

        // Return indices of inliers from fitted line with most inliers
        if (inliers.size() > inliersResult.size()) {
            inliersResult = inliers;
        }
    }

    return inliersResult;
}


// Use the pair object to hold your segmented results for the obstacle point cloud and the road point cloud
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(const typename pcl::PointCloud<PointT>::Ptr& cloud, int maxIterations, float distanceThreshold) {
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

	/*** Find inliers for the cloud. ***/

//    /*** PCL IMPLEMENTATION START ***/
//	// Create the segmentation object
//    pcl::SACSegmentation<PointT> seg;
//    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
//    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
//
//    // Optional
//    seg.setOptimizeCoefficients(true);
//    // Mandatory
//    seg.setModelType(pcl::SACMODEL_PLANE);
//    seg.setMethodType(pcl::SAC_RANSAC);
//    seg.setMaxIterations(maxIterations);
//    seg.setDistanceThreshold(distanceThreshold);
//
//    // Segment the largest planar component from the input cloud
//    seg.setInputCloud(cloud);
//    seg.segment(*inliers, *coefficients);
//
//    /*** PCL IMPLEMENTATION END ***/

    /*** STUDENT IMPLEMENTATION START ***/
    std::unordered_set<int> inliers_set = RansacPlane(cloud, maxIterations, distanceThreshold);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    for (auto i : inliers_set) {
        inliers->indices.push_back(i);
    }

    /*** STUDENT IMPLEMENTATION END ***/

    if (inliers->indices.empty()) {
        std::cerr << "Could not estimate a planar model for the given dataset" << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(const typename pcl::PointCloud<PointT>::Ptr& cloud, float clusterTolerance, int minSize, int maxSize) {
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    /*** Perform euclidean clustering to group detected obstacles ***/
    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for (const auto& get_indices : cluster_indices) {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>());

        for (const auto index : get_indices.indices) {
            cloud_cluster->points.push_back(cloud->points[index]);
        }

        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        clusters.push_back(cloud_cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(const typename pcl::PointCloud<PointT>::Ptr& cluster) {
    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box{};
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(const typename pcl::PointCloud<PointT>::Ptr& cloud, std::string file) {
    pcl::io::savePCDFileASCII(file, *cloud);
    std::cout << "Saved " << cloud->points.size () << " data points to " + file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file) {
    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    //load the pcd file
    if (-1 == pcl::io::loadPCDFile<PointT> (file, *cloud)) {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cout << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;
    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath) {
    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());
    return paths;
}