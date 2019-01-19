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
#include "Drawing/DrawCameraView.h"

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
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>

#include <algorithm>

#include <System/Thread.h>

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
void scene_clustering(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_filtered, std::stringstream& timestamp, float tolerance=0.02, float minsize=400, float maxsize = 50000);
using namespace cv;


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

	Mat RigidTrans = (Mat_<float>(2,3) << 0.340578906312217, 0.001079990228324099, -81.51090136055846, 0.0002631972723408978, 0.3441404069350427, 18.88198371718263);

	Mat iRigidTrans;

	cv::invertAffineTransform( RigidTrans, iRigidTrans );

    DrawDepthView ReadAndDrawDepth(CurrentWorkingFolder);
    ReadTimestampFile LocalisationData(CurrentWorkingFolder+"/robulab/Localization.timestamp");
	DrawCameraView ReadAndDrawCam(CurrentWorkingFolder);

	Mat ImageView( cvSize(CamWidth,CamHeight), CV_8UC3 );

    // Look at all data from timestamp
    while (ReadAndDrawDepth.GetNextTimestamp()) {
        // Get current data from Kinect
        if (!ReadAndDrawDepth.LoadFrame(ReadAndDrawDepth.CurrentTimestamp) ||
            !LocalisationData.GetDataForTimestamp(ReadAndDrawDepth.CurrentTimestamp) ||
            !ReadAndDrawCam.LoadFrame(ReadAndDrawDepth.CurrentTimestamp)) {
            // Should never appear
            fprintf(stderr, "argh\n\n");
            continue;
        }

		ReadAndDrawCam.Draw(ImageView,ReadAndDrawCam.FrameBuffer,1);

        // Load robot position
        uint16_t *LocalData = &((uint16_t *) ReadAndDrawDepth.FrameBuffer)[DepthHeight * DepthWidth - 1];
        struct robotPosition curentRobotPosition{};
        sscanf(LocalisationData.DataBuffer, "{\"x\":%f,\"y\":%f,\"o\":%f}",
               &curentRobotPosition.x, &curentRobotPosition.y, &curentRobotPosition.o);


		// Create a point cloud for this frame
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

		// PArse all depth point and project back to XYZ values
		for( int y = DepthHeight-1; y >= 0 ; y-- )
		{
			for( int x = DepthWidth-1; x >= 0 ; x-- )
			{
				float rx, rz, ry, ax, ay, az;

                if (*LocalData >= 500) // && *LocalData <= 4500 )
                {
                    int CurPixel = DepthWidth * y + x;

                    az = (*LocalData) / 1000.0f;    //The depth value from the Kinect back meters

					// a. is in kinect frame, r. is rotated
					rx = az;
					ax = ry = rx * DepthToCamera[CurPixel].X;
					ay = rz = rx * DepthToCamera[CurPixel].Y;

					std::vector<cv::Point2f> DepthPoint;
					DepthPoint.push_back(cv::Point2f(x,y));
					std::vector<cv::Point2f> RGBPoint;

					cv::transform( DepthPoint, RGBPoint, iRigidTrans );

					Point3_<uchar>* p = ImageView.ptr<Point3_<uchar> >(RGBPoint[0].y,RGBPoint[0].x);

					// construct a point
					pcl::PointXYZRGB Point;
					Point.x = -ry;
					Point.y = rx;
					Point.z = rz;
					Point.r = p->z;
					Point.g = p->y;
					Point.b = p->x;

                    // push it in the pont cloud
                    cloud->push_back(Point);
                }

                LocalData--;
            }
        }

        plane floorPlane;
//        initRot(cloud);
        removePlane(cloud, floorPlane);
        printf("Floor plane : a : %f, b: %f, c %f, d : %f \n", floorPlane.a, floorPlane.b, floorPlane.c, floorPlane.d);
        rotateCloud(cloud, floorPlane, curentRobotPosition);
//        pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
//        viewer.showCloud(cloud);
//        while (!viewer.wasStopped()) {}

        std::stringstream timestamp;
        timestamp << ReadAndDrawDepth.CurrentTimestamp.time+"_"+ReadAndDrawDepth.CurrentTimestamp.millitm;
        scene_clustering(cloud, timestamp);
    }

    return 0;

}

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

void scene_clustering(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_filtered, std::stringstream& timestamp, float tolerance, float minsize, float maxsize){
// Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (cloud_filtered);
    pcl::PCDWriter writer;

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;

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
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
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
        writer.write<pcl::PointXYZRGB> (filename.str (), *cloud_cluster, false); //*
        j++;
    }
}