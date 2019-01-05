/**
 * @file Main.cpp
 * @ingroup MobileRGBD Examples
 * @author Dominique Vaufreydaz, Grenoble Alpes University, Inria
 * @copyright All right reserved.
 */

#ifndef _FILE_OFFSET_BITS
#define _FILE_OFFSET_BITS  64
#endif

#include <System/ConfigSystem.h>

#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <stdio.h>

#include <sys/stat.h>
#if defined WIN32 || defined WIN64
	#include <direct.h>
#endif

#include "Drawing/DrawDepthView.h"

#include <System/SimpleList.h>
#include <System/SimpleString.h>
#include <System/ElapsedTime.h>
#include <System/TypedMemoryBuffer.h>

// Using namespaces
using namespace MobileRGBD;

// Internally we have also a MobileRGBD::Kinect1 namespace with classes to handle Kinect for Windows 1 devices
using namespace MobileRGBD::Kinect2;

#include <math.h>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <vtkRenderWindow.h>
#include <vtkPNGWriter.h>
#include <vtkWindowToImageFilter.h>
#include <vtkRenderWindowInteractor.h>

#include <pcl/visualization/cloud_viewer.h>

#include <algorithm>

#include <System/Thread.h>

void removePlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, float distanceThreshold = 0.03, int pointThreshold = 20000);
void scene_clustering(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_filtered, std::stringstream& timestamp, float tolerance = 0.02, float minsize = 400, float maxsize = 50000);
/** @brief You must call with a folder name
 * @return 0 if successful, negative value for failure.
 * @example
 */
int main( int argc, char *argv[] )
{
	// Get param without checking if param exist, tthis is a simple example
	Omiscid::SimpleString CurrentWorkingFolder = argv[1];

	// Read PointF matrix with projection coeffecient
	Omiscid::TypedMemoryBuffer<PointF> DepthToCamera;
	DepthToCamera.SetNewNumberOfElementsInBuffer( DepthWidth * DepthHeight );
	FILE * fdtc = fopen( "DepthToCameraTable.raw", "rb" );
	if ( fdtc == NULL || fread((PointF*)DepthToCamera, DepthWidth * DepthHeight * 8, 1, fdtc ) != 1 )
	{
		fprintf( stderr,"Unable to read DepthToCameraTable.raw\n" );
		return 0;
	}
	fclose( fdtc );


	DrawDepthView ReadAndDrawDepth(CurrentWorkingFolder);

	// Look at all data from timestamp
	while( ReadAndDrawDepth.GetNextTimestamp() )
	{
		// Get current data from Kinect
		if ( ReadAndDrawDepth.LoadFrame(ReadAndDrawDepth.CurrentTimestamp) == false )
		{
			// Should never appear
			fprintf( stderr, "argh\n\n" );
			continue;
		}


		// ok, got to depth frame, get it as uint16 data (millimeter value)
		uint16_t * LocalData = &((uint16_t*)ReadAndDrawDepth.FrameBuffer)[DepthHeight*DepthWidth-1];

		// Create a point cloud for this frame
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

		// PArse all depth point and project back to XYZ values
		for( int y = DepthHeight-1; y >= 0 ; y-- )
		{
			fprintf(stderr, "+");
			for( int x = DepthWidth-1; x >= 0 ; x-- )
			{
				float rx, rz, ry, ax, ay, az;

				if ( *LocalData >= 500 ) // && *LocalData <= 4500 )
				{
					int CurPixel = DepthWidth*y+x;

					az = (*LocalData)/1000.0f;	//The depth value from the Kinect back meters

					// a. is in kinect frame, r. is rotated
					rx = az;
					ax = ry = rx * DepthToCamera[CurPixel].X;
					ay = rz = rx * DepthToCamera[CurPixel].Y;

					// construct a point
					pcl::PointXYZ Point;
					Point.x = rz;
					Point.y = ry;
					Point.z = rx;

					// push it in the pont cloud
					cloud->push_back(Point);
				}

				LocalData--;
			}
		}
 		removePlane(cloud);

		std::stringstream timestamp;
		timestamp << ReadAndDrawDepth.CurrentTimestamp.time << "_" << ReadAndDrawDepth.CurrentTimestamp.millitm;
		scene_clustering(cloud, timestamp);

    // Here we have a new cloud, uncomment to view it
    /*pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    viewer.showCloud (cloud);
    while (!viewer.wasStopped ()) {}
		*/
    // Go within the loop to process next frame

	}

	return 0;

}

void removePlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, float distanceThreshold, int pointThreshold){
	/*
	 * remove plane from the point cloud.
	 */
    while (true) {
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        // Optional
        seg.setOptimizeCoefficients(true);
        // Mandatory
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(distanceThreshold);

        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);
        fprintf(stderr, "nb pt : %lu", inliers->indices.size(), "\n");
        if (inliers->indices.size() < pointThreshold){ break;} //break if the plane do not contain enough points

        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*cloud);
        }
}

void scene_clustering(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_filtered, std::stringstream& timestamp, float tolerance, float minsize, float maxsize){
// Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);
	pcl::PCDWriter writer;

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

	//Setting parameters
  ec.setClusterTolerance (tolerance);
  ec.setMinClusterSize (minsize);
  ec.setMaxClusterSize (maxsize);
  ec.setSearchMethod (tree);

	//Extracting clusters from the given cloud
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

	// Clusters are extracted, we iterate over them to save them
  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
		//Creating new cloud with individual cluster
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points.\n" << std::endl;

		//Creating new folder to save the clusters
		mkdir("ProcessedClusters", 0000700);
		std::stringstream filename;
		filename << "ProcessedClusters/" << timestamp.str();
		//Creating new folder for current timestamp
		mkdir(filename.str().c_str(), 0000700);
		//Changing filnename for each cluster at given timestamp
    filename << "/cloud_cluster_" << j << ".pcd";
		//Saving the cluster
    writer.write<pcl::PointXYZ> (filename.str (), *cloud_cluster, false); //*
    j++;
  }
}
