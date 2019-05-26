/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData() {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++) {
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
  	while(numOutliers--) {
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

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D() {
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene() {
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int maxIterations, float distanceTol) {
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	// For max iterations 
    for (int i = 0; i < maxIterations; i++) {
        // Randomly sample subset and fit line
        std::unordered_set<int> inliers;

        float x[2], y[2];
        int j = 0;
        // avoid picking the same point twice
        while(inliers.size() < 2) {
            auto index = rand() % (cloud->points.size());
            x[j] = cloud->points[index].x;
            y[j] = cloud->points[index].y;

            if (0 == inliers.count(index)) {
                inliers.insert(index);
                j++;
            }
        }

        float a = y[0] - y[1];
        float b = x[1] - x[0];
        float c = x[0] * y[1] - x[1] * y[0];
        // a constant number that will be used multiple times in following calculation
        float d2 = sqrt(a*a + b*b);

        // Measure distance between every point and fitted line
        for (int index = 0; index < cloud->points.size(); index++) {
            if (inliers.count(index) > 0) {
                continue;
            }

            auto point = cloud->points[index];
            float x = point.x;
            float y = point.y;

            float d = fabs(a*x + b*y + c) / d2;
            // If distance is smaller than threshold count it as inlier
            if (d <= distanceTol) {
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


std::unordered_set<int> RansacPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int maxIterations, float distanceTol) {
    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    // For max iterations
    for (int i = 0; i < maxIterations; i++) {
        // Randomly sample subset and fit line
        std::unordered_set<int> inliers;

        constexpr int kDimension = 3;
        float x[kDimension], y[kDimension], z[kDimension];
        int j = 0;
        // avoid picking the same point twice
        while(inliers.size() < kDimension) {
            auto index = rand() % (cloud->points.size());
            x[j] = cloud->points[index].x;
            y[j] = cloud->points[index].y;
            z[j] = cloud->points[index].z;

            if (0 == inliers.count(index)) {
                inliers.insert(index);
                j++;
            }
        }


        float v0[3] = {x[1] - x[0], y[1] - y[0], z[1] - z[0]};
        float v1[3] = {x[2] - x[0], y[2] - y[0], z[2] - z[0]};

        float a = v0[1] * v1[2] - v0[2] * v1[1];
        float b = v0[2] * v1[0] - v0[0] * v1[2];
        float c = v0[0] * v1[1] - v0[1] * v1[0];
        float d = -(a * x[0] + b * y[0] + c * z[0]);
        // a constant number that will be used multiple times in following calculation
        float d2 = sqrt(a*a + b*b + c*c);

        // Measure distance between every point and fitted line
        for (int index = 0; index < cloud->points.size(); index++) {
            if (inliers.count(index) > 0) {
                continue;
            }

            auto point = cloud->points[index];
            float x = point.x;
            float y = point.y;
            float z = point.z;

            float distance = fabs(a*x + b*y + c*z + d) / d2;
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



int main() {
	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
//    std::unordered_set<int> inliers = Ransac(cloud, 50, 0.5);

	// Set the max iteration and distance tolerance arguments for Ransac function
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
    std::unordered_set<int> inliers = RansacPlane(cloud, 50, 0.5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++) {
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

	// Render 2D point cloud with inliers and outliers
	if(inliers.size()) {
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else {
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped()) {
  	  viewer->spinOnce ();
  	}
  	
}
