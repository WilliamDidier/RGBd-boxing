#ifndef BODY_PROCESSING_H
#define BODY_PROCESSING_H

#include <pcl/features/moment_of_inertia_estimation.h>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <boost/thread/thread.hpp>
#include <iostream>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <stdio.h>


int compute_bounding_box(std::stringstream& timestamp, std::stringstream &filename);
int body_part_clustering (std::stringstream& timestamp, std::stringstream &filename);


#endif
