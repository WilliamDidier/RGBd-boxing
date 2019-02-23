#include "sceneClustering.h"

void scene_clustering(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_filtered, std::stringstream& timestamp, float tolerance, float minsize, float maxsize){
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (cloud_filtered);

    //Setting clustering parameters
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (tolerance);
    ec.setMinClusterSize (minsize);
    ec.setMaxClusterSize (maxsize);
    ec.setSearchMethod (tree);

    //Extracting clusters from the given cloud
    std::vector<pcl::PointIndices> cluster_indices;
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);

    //TODO : add actual body position.
    pcl::PointXYZ body_pos(0,0,0);

    // Clusters are extracted, we iterate over them to measure their distance to the body
    int j = 0;
		float distance[cluster_indices.size ()];
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        //Computing the cluster centroid
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid (*cloud_filtered, *it, centroid);
        std::cout << "Cluster centroid coordinates : " << centroid << std::endl;

				//Computing body-cluster distance
				distance[j] = pcl::euclideanDistance(body_pos, pcl::PointXYZ(centroid[0], centroid[1], centroid[2]));
				j++;
    }

    //Finding cluster closest to the body
		int min_dist_idx = distance[0] ;
    for (int i=1;  i < sizeof(distance)/sizeof(distance[0]);  ++i){
        if ( distance[i] < min_dist_idx){
             min_dist_idx = distance[i];
				}
		}

    //Creating a cloud corresponding to the closest cluster
		pcl::PointIndices it = cluster_indices[min_dist_idx];
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr body_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
		for (std::vector<int>::const_iterator pit = it.indices.begin (); pit != it.indices.end (); ++pit)
				body_cluster->points.push_back (cloud_filtered->points[*pit]);
		body_cluster->width = body_cluster->points.size ();
    body_cluster->height = 1;
    body_cluster->is_dense = true;

    //Saving the cloud
    pcl::PCDWriter writer;
		mkdir("ProcessedClusters", 0000700);
		std::stringstream filename;
		filename << "ProcessedClusters/" << timestamp.str();
    mkdir(filename.str().c_str(), 0000700);
    filename << "/" << timestamp.str() << "_body.pcd";
		writer.write<pcl::PointXYZRGB> (filename.str (), *body_cluster, false); //*
    compute_bounding_box(timestamp, filename);
		body_part_clustering(timestamp, filename);
}
