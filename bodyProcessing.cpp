#include "bodyProcessing.h"

int compute_bounding_box(std::stringstream& timestamp, std::stringstream &filename) {

    pcl::PCDWriter writer;

    // loading the file containing the body
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::io::loadPCDFile (filename.str(), *cloud);
		pcl::PointCloud<pcl::PointXYZ>::Ptr res (new pcl::PointCloud<pcl::PointXYZ>);

    // initializing the object that will compute the OBB
		pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
	  feature_extractor.setInputCloud(cloud);
	  feature_extractor.compute ();

	  std::vector <float> moment_of_inertia;
	  std::vector <float> eccentricity;
	  pcl::PointXYZ min_point_OBB;
	  pcl::PointXYZ max_point_OBB;
	  pcl::PointXYZ position_OBB;
	  Eigen::Matrix3f rotational_matrix_OBB;
	  float major_value, middle_value, minor_value;
	  Eigen::Vector3f major_vector, middle_vector, minor_vector;
	  Eigen::Vector3f mass_center;

    // extracting points for the OBB
	  feature_extractor.getMomentOfInertia (moment_of_inertia);
	  feature_extractor.getEccentricity (eccentricity);
	  feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
	  feature_extractor.getEigenValues (major_value, middle_value, minor_value);
	  feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
	  feature_extractor.getMassCenter (mass_center);

	  Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
	  Eigen::Quaternionf quat (rotational_matrix_OBB);

	  Eigen::Vector3f p1 (min_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
	  Eigen::Vector3f p2 (min_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
	  Eigen::Vector3f p3 (max_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
	  Eigen::Vector3f p4 (max_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
	  Eigen::Vector3f p5 (min_point_OBB.x, max_point_OBB.y, min_point_OBB.z);
	  Eigen::Vector3f p6 (min_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
	  Eigen::Vector3f p7 (max_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
	  Eigen::Vector3f p8 (max_point_OBB.x, max_point_OBB.y, min_point_OBB.z);

    // computing the true OBB points
	  p1 = rotational_matrix_OBB * p1 + position;
	  p2 = rotational_matrix_OBB * p2 + position;
	  p3 = rotational_matrix_OBB * p3 + position;
	  p4 = rotational_matrix_OBB * p4 + position;
	  p5 = rotational_matrix_OBB * p5 + position;
	  p6 = rotational_matrix_OBB * p6 + position;
	  p7 = rotational_matrix_OBB * p7 + position;
	  p8 = rotational_matrix_OBB * p8 + position;

    // storing the points into a points cloud
	  pcl::PointXYZ pt1 (p1 (0), p1 (1), p1 (2));
		res->points.push_back(pt1);
	  pcl::PointXYZ pt2 (p2 (0), p2 (1), p2 (2));
		res->points.push_back(pt2);
	  pcl::PointXYZ pt3 (p3 (0), p3 (1), p3 (2));
		res->points.push_back(pt3);
	  pcl::PointXYZ pt4 (p4 (0), p4 (1), p4 (2));
		res->points.push_back(pt4);
	  pcl::PointXYZ pt5 (p5 (0), p5 (1), p5 (2));
		res->points.push_back(pt5);
	  pcl::PointXYZ pt6 (p6 (0), p6 (1), p6 (2));
		res->points.push_back(pt6);
	  pcl::PointXYZ pt7 (p7 (0), p7 (1), p7 (2));
		res->points.push_back(pt7);
	  pcl::PointXYZ pt8 (p8 (0), p8 (1), p8 (2));
		res->points.push_back(pt8);

		res->width = 8;
		res->height = 1;
		res->is_dense = true;

    // saving the points cloud
    std::stringstream name;
    name << "ProcessedClusters/" << timestamp.str() << "/obb.pcd";
    writer.write<pcl::PointXYZ> (name.str(), *res, false);

		return(0);
}


int body_part_clustering (std::stringstream& timestamp, std::stringstream &filename)
{

    // loading the file containing the body
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl::io::loadPCDFile (filename.str(), *cloud);

    // initializing the search
    pcl::search::Search <pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);

    pcl::IndicesPtr indices (new std::vector <int>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 1.0);
    pass.filter (*indices);

    // segmentation
    pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
    reg.setInputCloud (cloud);
    reg.setIndices (indices);
    reg.setSearchMethod (tree);
    reg.setDistanceThreshold (10);
    reg.setPointColorThreshold (6.5);//6
    reg.setRegionColorThreshold (7.45);//5
    reg.setMinClusterSize (200);

    // extract the indices of each segmented cluster
    std::vector <pcl::PointIndices> color_clusters;
    reg.extract (color_clusters);

    pcl::PCDWriter writer;
    int j = 0;
    // iterating over the set of indices
    for (std::vector<pcl::PointIndices>::const_iterator it = color_clusters.begin (); it != color_clusters.end (); ++it)
    {
        // creating new cloud with individual cluster
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr body_part (new pcl::PointCloud<pcl::PointXYZRGB>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            body_part->points.push_back (cloud->points[*pit]); //*
        body_part->width = body_part->points.size ();
        body_part->height = 1;
        body_part->is_dense = true;
        std::cout << "PointCloud representing the Cluster: " << body_part->points.size () << " data points.\n" << std::endl;

        // saving the cluster into body_parts/body_part_X
        std::stringstream filename;
        filename << "ProcessedClusters/" << timestamp.str() << "/body_parts";
        //Creating new folder for current timestamp
        mkdir(filename.str().c_str(), 0000700);
        //Changing filnename for each cluster at given timestamp
        filename << "/body_part_" << j << ".pcd";
        //Saving the cluster
        writer.write<pcl::PointXYZRGB> (filename.str (), *body_part, false); //*

        j++;
    }

    return(0);

}
