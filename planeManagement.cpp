//
// Created by maxence on 19/01/19.
//

#include "planeManagement.h"
void initRot(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) {
    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    auto theta = static_cast<float>(-3.14/8) ;
    transform_2.rotate(Eigen::AngleAxisf(-0.643455, Eigen::Vector3f::UnitX()));

    pcl::transformPointCloud(*cloud, *cloud, transform_2);
}

void rotateCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, plane &floorPlane, struct robotPosition &robPos){
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();

    auto costheta = static_cast<float>(floorPlane.c/(sqrt(pow(floorPlane.b,2)+pow(floorPlane.c,2))));
    if (floorPlane.c < 0){
        costheta = -costheta;
    }

    printf(" theta : %f \n", costheta);
    float theta = static_cast<float>(-acos(costheta));
    printf(" theta : %f \n", theta);
    transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitX()));
    pcl::transformPointCloud (*cloud, *cloud, transform);
    transform = Eigen::Affine3f::Identity();

    transform.translation() << 0.02, 0, (floorPlane.d/(sqrt(pow(floorPlane.b,2)+pow(floorPlane.c,2)+pow(floorPlane.a,2))));
    pcl::transformPointCloud (*cloud, *cloud, transform);
    transform = Eigen::Affine3f::Identity();


    transform.rotate (Eigen::AngleAxisf (-robPos.o, Eigen::Vector3f::UnitZ()));
    pcl::transformPointCloud (*cloud, *cloud, transform);
    transform = Eigen::Affine3f::Identity();

    transform.translation() << -robPos.x, -robPos.y, 0;
    pcl::transformPointCloud (*cloud, *cloud, transform);
}

void removePlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, plane &bestPlane, float distanceThreshold, int pointThreshold){
	/*
	 * remove plane from the point cloud.
	 */
    bestPlane.c = 0;
    while (true) {
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        // Optional
        seg.setOptimizeCoefficients(true);
        // Mandatory
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(distanceThreshold);

        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);
        fprintf(stderr, "nb pt : %lu \n", inliers->indices.size());
        if (inliers->indices.size() < pointThreshold){ break;} //break if the plane do not contain enough points
        std::cerr << "Model coefficients: " << coefficients->values[0] << " "
                  << coefficients->values[1] << " "
                  << coefficients->values[2] << " "
                  << coefficients->values[3] << std::endl;
        if (abs(coefficients->values[2]) > abs(bestPlane.c)){
            bestPlane.a = coefficients->values[0];
            bestPlane.b = coefficients->values[1];
            bestPlane.c = coefficients->values[2];
            bestPlane.d = coefficients->values[3];
        }
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*cloud);
        }

}
