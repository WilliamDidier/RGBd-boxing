//
// Created by maxence on 19/01/19.
//

#ifndef TEST_PLANEMANAGEMENT_H
#define TEST_PLANEMANAGEMENT_H

#include <pcl/common/common.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

struct plane{
    /*
     * Representation of a plane
     * ax+by+cz+d = 0
     */
    float a;
    float b;
    float c;
    float d;
};

struct robotPosition{
    /*
     * Store the robot position
     */
    float x;
    float y;
    float o;
};

void removePlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, struct plane &bestPlane, float distanceThreshold = 0.03, int pointThreshold = 20000);
void rotateCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, struct plane &bestPlane, struct robotPosition &robPos);
void initRot(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);

#endif //TEST_PLANEMANAGEMENT_H
