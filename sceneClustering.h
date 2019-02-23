#ifndef SCENE_CLUSTERING_H
#define SCENE_CLUSTERING_H

#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/distances.h>
#include <pcl/common/centroid.h>
#include <pcl/io/pcd_io.h>
#include "bodyProcessing.h"

void scene_clustering(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_filtered, std::stringstream& timestamp, float tolerance=0.02, float minsize=400, float maxsize = 50000);

#endif
