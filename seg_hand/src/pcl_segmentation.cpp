#include "seg_hand/pcl_segmentation.h"

//class PCL_SEGMENTATION;
pcl::PointCloud<pcl::PointXYZ> PCL_SEGMENTATION::_voxel_grid(pcl::PointCloud<pcl::PointXYZ>::Ptr input)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_vg(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(input);
    vg.setLeafSize(0.01f,0.01f,0.01f);
    vg.filter(*cloud_vg);

    return *cloud_vg;

}

pcl::PointCloud<pcl::PointXYZ> PCL_SEGMENTATION::_depth_segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr input,std::string filter_field_name, float min_limit, float max_limit)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_dep(new pcl::PointCloud<pcl::PointXYZ>);
    //segmentation based on depth(z-axis)
    //create the filter
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(input);
    pass.setFilterFieldName(filter_field_name);  
    
    //identify  the closest set of points (z ranges widely)
    pass.setFilterLimits(min_limit, max_limit);
    pass.setFilterLimitsNegative(false);
    pass.filter(*cloud_dep);
    return *cloud_dep;
        
}

pcl::PointCloud<pcl::PointXYZ> PCL_SEGMENTATION::_comparison_segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr input, std::string comparison_field, float upper_limit, float lower_limit)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_com(new pcl::PointCloud<pcl::PointXYZ>);
    // use conditional segmentation
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZ> ());
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ> ::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> (comparison_field, pcl::ComparisonOps:: GT, lower_limit)));
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ> ::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> (comparison_field, pcl::ComparisonOps:: LT, upper_limit)));
    //can also add condition along y-axis
        
    pcl::ConditionalRemoval<pcl::PointXYZ> condrm;
    condrm.setInputCloud(input);
    condrm.setCondition(range_cond);
    condrm.setKeepOrganized(false);
    condrm.filter(*cloud_com);

    return *cloud_com;
}

std::vector<pcl::PointIndices> _ktree_indices(pcl::PointCloud<pcl::PointXYZ>::Ptr input, float cluster_tolerance)
{
    //creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>:: Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(input);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setInputCloud(input);
    ec.setClusterTolerance(cluster_tolerance);
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(50000);
    ec.setSearchMethod(tree);
    ec.extract(cluster_indices);

    return cluster_indices;
}

pcl::PointXYZ PCL_SEGMENTATION::_cluster_centroid(pcl::PointCloud<pcl::PointXYZ>::Ptr input)
{
    pcl::CentroidPoint<pcl::PointXYZ> input_centroid;
    for (size_t i = 0; i < input -> points.size(); ++i)
    {
        input_centroid.add (input-> points[i]);
    }
    pcl::PointXYZ cluster_centroid;
    input_centroid.get(cluster_centroid);

    return cluster_centroid;
}

pcl::PointCloud<pcl::PointXYZ> PCL_SEGMENTATION::_cluster_selection(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr ref_input, std::vector<pcl::PointIndices> cluster_indices)
{
    //cluster the remaining point cloud, select the cluster which contains object
    pcl::PointXYZ ref_centr;    
    ref_centr = _cluster_centroid(ref_input); 

    pcl::PointCloud<pcl::PointXYZ>:: Ptr selected_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    float min_dis = 0.0;
    int cluster_num = 0;
    int j=0;
    for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>:: Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>:: const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
                cloud_cluster->points.push_back(input->points[*pit]);
        }

        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        //calculate the centroid for each cluster
        pcl::PointXYZ cluster_centr = _cluster_centroid(cloud_cluster);

        if(min_dis >= pcl::euclideanDistance(ref_centr,cluster_centr))
        {
            min_dis = pcl::euclideanDistance(ref_centr, cluster_centr);
            cluster_num = j;
        }
           
        std::stringstream ss;
        ss << "cloud_cluster"<< j <<".pcd";
        pcl::PCDWriter writer;
        writer.write<pcl::PointXYZ> (ss.str(), *cloud_cluster, false); 
            
            
        j++;
            
        }
    
        std::stringstream ss;
        ss << "cloud_cluster" << cluster_num << ".pcd";
        pcl::PCDReader reader;
        reader.read(ss.str(),*selected_cloud);
       
        //writer.write<pcl::PointXYZ>("object.pcd", *cloud_h_o, false);
        return *selected_cloud;
}

pcl::PointCloud<pcl::PointXYZ> PCL_SEGMENTATION::_box_segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr ref_input)
{
    
    //get the range of the reference input point cloud
    /*
    pcl::PointXYZ minPT, maxPT;
    pcl::getMinMax3D(*ref_input, minPT, maxPT);
    */
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_x(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(input);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(minPT.x, maxPT.x);
    pass.setFilterLimitsNegative(false);
    pass.filter(*cloud_x);

    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_box(new pcl::PointCloud<pcl::PointXYZ>);
    pass.setInputCloud(cloud_x);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(minPT.y, maxPT.y);
    pass.filter(*cloud_box);

    return *cloud_box;

}

pcl::PointCloud<pcl::PointXYZ> PCL_SEGMENTATION::_seg_hand_out(pcl::PointCloud<pcl::PointXYZ>::Ptr scene_point_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr hand_point_cloud)
{
    //parameter initialization
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_vg(new pcl::PointCloud<pcl::PointXYZ>),
                                        cloud_dep(new pcl::PointCloud<pcl::PointXYZ>),
                                        cloud_com(new pcl::PointCloud<pcl::PointXYZ>),
                                        cloud_clus(new pcl::PointCloud<pcl::PointXYZ>),
                                        cloud_box(new pcl::PointCloud<pcl::PointXYZ>),
                                        cloud_obj(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<pcl::PointIndices> cluster_indices;

  

    _get_minmax3D(*hand_point_cloud);
    *cloud_vg = PCL_SEGMENTATION::_voxel_grid(scene_point_cloud);  
    *cloud_dep = PCL_SEGMENTATION::_depth_segmentation(cloud_vg, "z", minPT.z - 0.05, minPT.z + 0.05);
    *cloud_com = PCL_SEGMENTATION::_comparison_segmentation(cloud_dep,"x", 0.1,-0.1);
    cluster_indices = PCL_SEGMENTATION::_ktree_indices(cloud_com, 0.2);
    *cloud_clus = _cluster_selection(cloud_com, hand_point_cloud, cluster_indices);
    //*cloud_box = _box_segmentation(cloud_clus, hand_point_cloud);
    //return *cloud_box;
    return *cloud_clus;
   


}

void PCL_SEGMENTATION::_get_minmax3D(pcl::PointCloud<pcl::PointXYZ> input)
{
    pcl::getMinMax3D(input, minPT, maxPT);
}