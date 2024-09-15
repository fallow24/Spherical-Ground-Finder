#include "ground_finder.h"

void GroundFinder::initMarker(){
    n_marker.type = visualization_msgs::Marker::ARROW;
    n_marker.action = visualization_msgs::Marker::ADD;
    n_marker.pose.orientation.x = 0.0;
    n_marker.pose.orientation.y = 0.0;
    n_marker.pose.orientation.z = 0.0;
    n_marker.pose.orientation.w = 1.0;
    n_marker.scale.x = 0.02;
    n_marker.scale.y = 0.06;
    n_marker.scale.z = 0.06;
    n_marker.color.a = 1.0;
    n_marker.color.r = 1.0;
    n_marker.color.g = 0.0;
    n_marker.color.b = 0.0;
}

int64_t GroundFinder::delete_points(pcl::PointCloud<PointType>::Ptr &cur_scan, pcl::PointIndices::Ptr &indices, const bool negativ){
    pcl::ExtractIndices<PointType> extract;
    auto start_del    = std::chrono::high_resolution_clock::now();
    extract.setInputCloud(cur_scan);
    extract.setIndices(indices);
    extract.setNegative(negativ);
    extract.filter(*cur_scan);
    auto end_del      = std::chrono::high_resolution_clock::now();
    auto duration_del = std::chrono::duration_cast<std::chrono::microseconds>(end_del - start_del).count();
    if(!quiet) ROS_INFO("Delete points from cloud takes: %0.3f ms", float(duration_del) / 1e3);

    return duration_del;
}

int64_t GroundFinder::downsample(pcl::PointCloud<PointType>::Ptr &cur_scan){
    auto start_ds = std::chrono::high_resolution_clock::now();
    pcl::VoxelGrid<PointType> ds;
    ds.setInputCloud(cur_scan);
    ds.setLeafSize(ds_size, ds_size, ds_size);
    ds.filter(*cur_scan);
    auto end_ds = std::chrono::high_resolution_clock::now();
    auto duration_ds = std::chrono::duration_cast<std::chrono::microseconds>(end_ds - start_ds).count();
    if(!quiet) ROS_INFO("Downsampling cloud takes: %0.6f ms", float(duration_ds)/ 1e3);

    // Write to file
    if(write2file) csv << duration_ds << ",";

    return duration_ds;
}

int64_t GroundFinder::filter_cloud_geo(pcl::PointCloud<PointType>::Ptr &cur_scan){
    pcl::PointIndices::Ptr del_points(new pcl::PointIndices());
    del_points->indices.resize(cur_scan->points.size());

    // Helper variables
    int count_dp = 0;
    double d_sqr = 0.0;

    // Loop through cloud
    auto start_loop = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < (*cur_scan).size(); i++){
        PointType p_i = cur_scan->points[i];
        // Check if point lies within radius to be filtered out
        d_sqr = p_i.x * p_i.x + p_i.y * p_i.y + p_i.z * p_i.z;
        if(d_sqr <= radius_filter * radius_filter){
            del_points->indices[count_dp++] = i;
        }
    }
    del_points->indices.resize(count_dp);
    auto end_loop = std::chrono::high_resolution_clock::now();
    auto duration_loop = std::chrono::duration_cast<std::chrono::microseconds>(end_loop - start_loop).count();
    if(!quiet) ROS_INFO("Looping through cloud takes: %0.6f ms", float(duration_loop)/ 1e3);

    // Delete points from cloud
    auto duration_del = delete_points(cur_scan, del_points, true);

    // Write to file
    if(write2file) csv << "0," << duration_loop << "," << duration_del << ",";

    return duration_loop + duration_del;
}

int64_t GroundFinder::filter_cloud_kdt(pcl::PointCloud<PointType>::Ptr &cur_scan){
    // Initialize + Build balanced kd-tree from point cloud
    pcl::KdTreeFLANN<PointType> kd_tree_del;
    auto start_build    = std::chrono::high_resolution_clock::now();
    kd_tree_del.setInputCloud(cur_scan);
    auto end_build      = std::chrono::high_resolution_clock::now();
    auto duration_build = std::chrono::duration_cast<std::chrono::microseconds>(end_build - start_build).count();
    if(!quiet) ROS_INFO("Building tree takes: %0.3f ms", float(duration_build) / 1e3);

    // Radius search
    PointType origin;
    origin.x = 0.0;
    origin.y = 0.0;
    origin.z = 0.0;
    std::vector<int> search_radius_result;
    std::vector<float> search_radius_distances;
    auto start_search    = std::chrono::high_resolution_clock::now();
    kd_tree_del.radiusSearch(origin, radius_filter, search_radius_result, search_radius_distances);
    auto end_search      = std::chrono::high_resolution_clock::now();
    auto duration_search = std::chrono::duration_cast<std::chrono::microseconds>(end_search - start_search).count();
    if(!quiet) ROS_INFO("Radius search takes: %0.3f ms", float(duration_search) / 1e3);

    // Delete points from cloud
    pcl::PointIndices::Ptr del_points(new pcl::PointIndices());
    del_points->indices = search_radius_result;
    auto duration_del = delete_points(cur_scan, del_points, true);

    // Write to file
    if(write2file) csv << duration_build << "," << duration_search << "," << duration_del << ",";

    return duration_build + duration_search + duration_del;
}

int64_t GroundFinder::create_subcloud_geo(pcl::PointCloud<PointType>::Ptr &cur_scan, PointType query_point){
    pcl::PointIndices::Ptr points(new pcl::PointIndices());
    pcl::PointIndices::Ptr points_backup(new pcl::PointIndices());
    points->indices.resize(cur_scan->points.size());
    points_backup->indices.resize(cur_scan->points.size());

    // Helper variables
    std::vector<double> d = {0.0, 0.0, 0.0};
    double d_sqr = 0.0;
    int count_p = 0;
    int count_pb = 0;

    // Loop through cloud
    auto start_loop = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < (*cur_scan).size(); i++){
        PointType p_i = cur_scan->points[i];
        // distance squared to query point
        subtract_points(p_i, query_point, d);
        d_sqr = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
        // Check if point lies within sphere around the query point
        if(d_sqr <= radius_subcloud * radius_subcloud){
            points_backup->indices[count_pb++] = i;
            // Now check if also point lies within cheese form (= sphere around query point with height limit)
            if(fabs(dot_product(d, n)) <= height_subcloud*0.5){
                points->indices[count_p++] = i;
            }
        }
    }
    points->indices.resize(count_p);
    points_backup->indices.resize(count_pb);
    auto end_loop = std::chrono::high_resolution_clock::now();
    auto duration_loop = std::chrono::duration_cast<std::chrono::microseconds>(end_loop - start_loop).count();
    if(!quiet) ROS_INFO("Looping through cloud takes: %0.6f ms", float(duration_loop)/ 1e3);

    // Delete points from cloud
    int64_t duration_del;
    if(points->indices.size() < 3){
        // Not enough points found due to bad cheese form or bad n vector -> take whole sphere
        duration_del = delete_points(cur_scan, points_backup);
    } else {
        duration_del = delete_points(cur_scan, points);
    }

    // Write to file
    if(write2file) csv << "0," << duration_loop << "," << duration_del << ",";

    return duration_loop + duration_del;
}

int64_t GroundFinder::create_subcloud_kdt(pcl::PointCloud<PointType>::Ptr &cur_scan, int k, PointType query_point){
    // Build balanced kd-tree from (filtered) point cloud
    pcl::KdTreeFLANN<PointType> kd_tree;
    auto start_build    = std::chrono::high_resolution_clock::now();
    kd_tree.setInputCloud(cur_scan);
    auto end_build      = std::chrono::high_resolution_clock::now();
    auto duration_build = std::chrono::duration_cast<std::chrono::microseconds>(end_build - start_build).count();
    if(!quiet) ROS_INFO("Building tree takes: %0.6f ms", float(duration_build) / 1e3);

    // KNN search
    std::vector<int> search_result;
    std::vector<float> search_distances;
    auto start_search    = std::chrono::high_resolution_clock::now();
    kd_tree.nearestKSearch(query_point, k, search_result, search_distances); // NOTE: not ranged search!
    auto end_search      = std::chrono::high_resolution_clock::now();
    auto duration_search = std::chrono::duration_cast<std::chrono::microseconds>(end_search - start_search).count();
    if(!quiet) ROS_INFO("Search kNN takes: %0.6f ms", float(duration_search) / 1e3);

    // Delete Points from cloud
    pcl::PointIndices::Ptr sub_points(new pcl::PointIndices());
    sub_points->indices = search_result;
    auto duration_del = delete_points(cur_scan, sub_points);

    // Write to file
    if(write2file) csv << duration_build << "," << duration_search << "," << duration_del << ",";

    return duration_build + duration_search + duration_del;
}

int64_t GroundFinder::filter_create_subcloud_geo(pcl::PointCloud<PointType>::Ptr &cur_scan, PointType query_point){
    pcl::PointIndices::Ptr points(new pcl::PointIndices());
    pcl::PointIndices::Ptr points_backup(new pcl::PointIndices());
    points->indices.resize(cur_scan->points.size());
    points_backup->indices.resize(cur_scan->points.size());

    // Helper variables
    std::vector<double> d = {0.0, 0.0, 0.0};
    double d1_sqr = 0.0, d2_sqr = 0.0;
    int count_p = 0;
    int count_pb = 0;

    // Loop through cloud
    auto start_loop = std::chrono::high_resolution_clock::now();
    for(int i = 0; i < (*cur_scan).size(); i++){
        PointType p_i = cur_scan->points[i];
        // distance squared to center (0,0,0)
        d1_sqr = p_i.x * p_i.x + p_i.y * p_i.y + p_i.z * p_i.z;
        // distance squared to query point
        subtract_points(p_i, query_point, d);
        d2_sqr = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
        // Check if point lies outside of filter radius and within sphere around the query point
        if(d1_sqr > radius_filter * radius_filter && d2_sqr <= radius_subcloud * radius_subcloud){
            points_backup->indices[count_pb++] = i;
            // Now check if also point lies within cheese form (= sphere around query point with height limit)
            if(fabs(dot_product(d, n)) <= height_subcloud*0.5){
                points->indices[count_p++] = i;
            }
        }
    }
    points->indices.resize(count_p);
    points_backup->indices.resize(count_pb);
    auto end_loop = std::chrono::high_resolution_clock::now();
    auto duration_loop = std::chrono::duration_cast<std::chrono::microseconds>(end_loop - start_loop).count();
    if(!quiet) ROS_INFO("Looping through cloud takes: %0.6f ms", float(duration_loop)/ 1e3);

    // Delete points from cloud
    int64_t duration_del;
    if(points->indices.size() < 3){
        // Not enough points found due to bad cheese form or bad n vector -> take whole sphere
        duration_del = delete_points(cur_scan, points_backup);
    } else {
        duration_del = delete_points(cur_scan, points);
    }

    // Write to file
    if(write2file) csv << "0,0,0,0," << duration_loop << "," << duration_del << ",";

    return duration_loop + duration_del;
}

int64_t GroundFinder::determine_n_ground_plane(pcl::PointCloud<PointType>::Ptr &cur_scan, PlaneSegm type, geometry_msgs::Vector3Stamped &n_msg){
    int64_t duration_plane = 0.0;
    // Select Plane algorithm
    switch(type){
        case LSF:{
            float curv;
            Eigen::Vector4f params;
            auto start_plane = std::chrono::high_resolution_clock::now();
            pcl::computePointNormal(*cur_scan, params, curv); // NOTE: eigenvector = already normalized :)
            // Set new normal vector
            n[0] = params[0];
            n[1] = params[1];
            n[2] = params[2];
            if(!convert_n_to_map_frame(n_msg)) return -1000;
            auto end_plane = std::chrono::high_resolution_clock::now();
            duration_plane = std::chrono::duration_cast<std::chrono::microseconds>(end_plane - start_plane).count();
            break;
        }
        case PCA:{
            pcl::PCA<PointType> pca;
            auto start_plane = std::chrono::high_resolution_clock::now();
            pca.setInputCloud(cur_scan);
            Eigen::Matrix3f eigen_vecs = pca.getEigenVectors(); // NOTE: eigenvector = already normalized :)
            // Set new normal vector
            n[0] = eigen_vecs.coeff(0, 2);
            n[1] = eigen_vecs.coeff(1, 2);
            n[2] = eigen_vecs.coeff(2, 2);
            if(!convert_n_to_map_frame(n_msg)) return -1000;
            auto end_plane = std::chrono::high_resolution_clock::now();
            duration_plane = std::chrono::duration_cast<std::chrono::microseconds>(end_plane - start_plane).count();
            break;
        }
        case RANSAC:{
            // NOTE: unter 0.01 kaum besseres ergebnis dafür langsamer, über 0.02 deutlich ungenauer und erst wirklich schneller ab 0.05 (sehr ungenau schon)
            double dist_thresh = 0.01;
            std::vector<int> inliers;
            pcl::PointIndices::Ptr inliers_ptr (new pcl::PointIndices());
            // pcl::PointCloud<PointType>::Ptr tmp(new pcl::PointCloud<PointType>);
            // float curv;
            // Eigen::Vector4f params;

            auto start_plane = std::chrono::high_resolution_clock::now();
            for(int i = 0; i < max_iterations_plane_detection; i++){
                // Created RandomSampleConsensus object the appropriate model
                pcl::SampleConsensusModelPlane<PointType>::Ptr model(new pcl::SampleConsensusModelPlane<PointType>(cur_scan));
                pcl::RandomSampleConsensus<PointType> ransac(model);
                ransac.setDistanceThreshold(dist_thresh);
                // NOTE: max_iterations_ = def: 10000, but = attribute from parent class; not used in ransac! = no effect just propability!
                // NOTE: Propability (def:0.99) - basically no effect on speed and insignificant one on accuracy (worse at propability < 0.5)!
                // PCA for normal vector calculation
                pcl::PCA<PointType> pca;
                try{
                    ransac.computeModel();
                    ransac.getInliers(inliers);

                    /* LSF */
                    // pcl::copyPointCloud(*cur_scan, inliers, *tmp);
                    // pcl::computePointNormal(*tmp, params, curv); // NOTE: eigenvector = already normalized :)
                    // // Set new normal vector
                    // n[0] = params[0];
                    // n[1] = params[1];
                    // n[2] = params[2];

                    /* PCA */
                    inliers_ptr->indices = inliers;
                    pca.setInputCloud(cur_scan);
                    pca.setIndices(inliers_ptr);
                    Eigen::Matrix3f eigen_vecs = pca.getEigenVectors(); // NOTE: eigenvector = already normalized :)
                    // Set new normal vector
                    n[0] = eigen_vecs.coeff(0, 2);
                    n[1] = eigen_vecs.coeff(1, 2);
                    n[2] = eigen_vecs.coeff(2, 2);
                } catch(pcl::InitFailedException){
                    return -1000;
                }

                bool last_iteration = (i == max_iterations_plane_detection - 1);
                // Check if ground plane found
                if(convert_n_to_map_frame(n_msg, last_iteration)) break;
                // Check if last try failed (found wall @ max iterations)
                else if(last_iteration) return -1000;

                // If not remove all points from cloud and re-do RANSAC alg
                delete_points(cur_scan, inliers_ptr, true);
                inliers = {};
                inliers_ptr->indices = {};
                // If not enough points left to fit plane = stop plane detection
                if(cur_scan->size() < 3) return -1000;
            }

            auto end_plane = std::chrono::high_resolution_clock::now();
            duration_plane = std::chrono::duration_cast<std::chrono::microseconds>(end_plane - start_plane).count();
            break;
        }
        case RHT:{
            // Accumulator (90 and 180 = 2° accuracy)
            int rhoNum = 7, phiNum = 90, thetaNum = 180, rhoMax = 5, accumulatorMax = 10;
            // NOTE: rhoMax = max distance from plane to origin of frame (in our case = radius sphere + little wiggle room with int = 1)
            // NOTE: rhoNum = no effect on accuracy but makes it faster if low! now = 7 -> roughly speed of ran
            // NOTE: phiNum and thetaNum define accuracy of plane orientation! but higher = slower!
            Accumulator acc(rhoNum, phiNum, thetaNum, rhoMax, accumulatorMax);
            // Hough object
            double minDist = 0; // min distance between points
            double maxDist = std::numeric_limits<double>::max(); // max distance between points

            auto start_plane = std::chrono::high_resolution_clock::now();
            for(int i = 0; i < max_iterations_plane_detection; i++){
                Hough hough(cur_scan, &acc, minDist, maxDist);
                double rho = hough.RHT(n); // NOTE: eigenvector = already normalized :)

                bool last_iteration = (i == max_iterations_plane_detection - 1);
                // Check if ground plane found
                if(convert_n_to_map_frame(n_msg, last_iteration) && rho != -1) break;
                // Check if last try failed (found wall or no plane detected with rht @ max iterations)
                else if (last_iteration) return -1000;

                // Check if try failed
                if(rho != -1){
                    // If not remove all points from cloud and re-do RHT alg
                    pcl::PointIndices::Ptr del_points(new pcl::PointIndices());
                    del_points->indices.resize(cur_scan->points.size());
                    int count = 0;

                    for (int i = 0; i < (*cur_scan).size(); i++){
                        PointType p_i = cur_scan->points[i];
                        double distance = fabs(p_i.x * n[0] + p_i.y * n[1] + p_i.z * n[2] - rho);
                        // Check if point lies on wrongly detected plane
                        if(distance <= 0.05){
                            del_points->indices[count++] = i;
                        }
                    }
                    del_points->indices.resize(count);
                    delete_points(cur_scan, del_points, true);
                    // If not enough points left to fit plane = stop plane detection
                    if(cur_scan->size() < 3) return -1000;

                    // TODO comment out!
                    // Publish new subcloud for next try
                    // sensor_msgs::PointCloud2::Ptr sub_cloud_msg(new sensor_msgs::PointCloud2());
                    // pcl::toROSMsg(*cur_scan, *sub_cloud_msg);
                    // sub_cloud_msg->header.stamp = n_msg.header.stamp;
                    // sub_cloud_msg->header.frame_id = "pandar_frame";
                    // pub_test2.publish(*sub_cloud_msg);
                }
            }

            auto end_plane = std::chrono::high_resolution_clock::now();
            duration_plane = std::chrono::duration_cast<std::chrono::microseconds>(end_plane - start_plane).count();
            break;
        }
        case RHT2:{
            // Accumulator (36 and 72 = 5° accuracy)
            int rhoNum = 7, phiNum = 36, thetaNum = 72, rhoMax = 5, accumulatorMax = 10;
            Accumulator acc(rhoNum, phiNum, thetaNum, rhoMax, accumulatorMax);
            // Hough object
            double minDist = 0; // min distance between points
            double maxDist = std::numeric_limits<double>::max(); // max distance between points

            std::vector<double> temp_n = {0.0, 0.0, 0.0};

            auto start_plane = std::chrono::high_resolution_clock::now();
            for(int i = 0; i < max_iterations_plane_detection; i++){
                bool last_iteration = (i == max_iterations_plane_detection - 1);

                Hough hough(cur_scan, &acc, minDist, maxDist);
                double rho = hough.RHT(temp_n); // NOTE: eigenvector = already normalized :)

                pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
                inliers->indices.resize(cur_scan->points.size());
                int count = 0;

                try{
                    // Fit plane if a plane was found (rho != 1)
                    if(rho != -1){
                        // Determine inliers of found rough plane
                        for (int i = 0; i < (*cur_scan).size(); i++){
                            PointType p_i = cur_scan->points[i];
                            double distance = fabs(p_i.x * n[0] + p_i.y * n[1] + p_i.z * n[2] - rho);
                            // Check if point lies on detected plane
                            if(distance <= 0.05){
                                inliers->indices[count++] = i;
                            }
                        }
                        inliers->indices.resize(count);
                        // Check if enough points for PCA if not just retry RHT
                        if(!quiet) ROS_WARN("Inliers num: %ld", inliers->indices.size());
                        if(!last_iteration && count < 3) continue;

                        // PCA for normal vector calculation
                        pcl::PCA<PointType> pca;
                        pca.setInputCloud(cur_scan);
                        pca.setIndices(inliers);

                        // TODO comment out!
                        // // Publish detected inliers
                        // sensor_msgs::PointCloud2::Ptr sub_cloud_msg(new sensor_msgs::PointCloud2());
                        // pcl::PointCloud<PointType>::Ptr final(new pcl::PointCloud<PointType>);
                        // pcl::copyPointCloud(*cur_scan, inliers->indices, *final);
                        // pcl::toROSMsg(*final, *sub_cloud_msg);
                        // sub_cloud_msg->header.stamp = n_msg.header.stamp;
                        // sub_cloud_msg->header.frame_id = "pandar_frame";
                        // pub_test.publish(*sub_cloud_msg);

                        Eigen::Matrix3f eigen_vecs = pca.getEigenVectors(); // NOTE: eigenvector = already normalized :)
                        // Set new normal vector
                        n[0] = eigen_vecs.coeff(0, 2);
                        n[1] = eigen_vecs.coeff(1, 2);
                        n[2] = eigen_vecs.coeff(2, 2);
                    }
                } catch(pcl::InitFailedException){
                    ROS_ERROR("Not enough points for PCA!");
                    convert_n_to_map_frame(n_msg, true);
                    return -1000;
                }

                // Check if ground plane found
                if(convert_n_to_map_frame(n_msg, last_iteration) && rho != -1) break;
                // Check if last try failed (found wall or no polane detected with rht @ max iterations)
                else if (last_iteration) return -1000;

                // If not remove those points from cloud and re-do RHT alg
                if(rho != -1){
                    delete_points(cur_scan, inliers, true);
                    // If not enough points left to fit plane = stop plane detection
                    if(cur_scan->size() < 3){
                        ROS_ERROR("Not enough points after delete in cur_scan! num_points < 3");
                        convert_n_to_map_frame(n_msg, true);
                        return -1000;
                    }

                    // TODO comment out!
                    // // Publish (filtered) subcloud
                    // sensor_msgs::PointCloud2::Ptr sub_cloud_msg(new sensor_msgs::PointCloud2());
                    // pcl::toROSMsg(*cur_scan, *sub_cloud_msg);
                    // sub_cloud_msg->header.stamp = n_msg.header.stamp;
                    // sub_cloud_msg->header.frame_id = "pandar_frame";
                    // pub_test2.publish(*sub_cloud_msg);
                }
            }

            auto end_plane = std::chrono::high_resolution_clock::now();
            duration_plane = std::chrono::duration_cast<std::chrono::microseconds>(end_plane - start_plane).count();
            break;
        }
    }

    if(!quiet) {
        ROS_INFO("Plane Segmentation takes: %0.6f ms", float(duration_plane) / 1e3);
        // ROS_INFO("Normal Vector: nx: %.3f, ny: %.3f, nz: %.3f", n[0], n[1], n[2]);
    }
    return duration_plane;
}

bool GroundFinder::convert_n_to_map_frame(geometry_msgs::Vector3Stamped &n_msg, const bool &last_iteration){
    geometry_msgs::TransformStamped t;
    try {
        // Listen to tf tree for transformation
        t = tf_buffer.lookupTransform("map2", "pandar_frame", ros::Time(0));
    } catch(tf2::TransformException& ex){
        ROS_ERROR("Failed to listen to tf tree!\n\n");
        return false;
    }
    // Create Vector n in pandar frame
    geometry_msgs::Vector3Stamped n_pandar;
    n_pandar.vector.x = n[0];
    n_pandar.vector.y = n[1];
    n_pandar.vector.z = n[2];

    tf2::doTransform(n_pandar, n_msg, t);

    // Make sure it always points into ground // NOTE: Assumption = abs(slope of ground plane) < 45°
    std::vector<double> down = {0.0, 0.0, -1.0};
    std::vector<double> n_map2 = {n_msg.vector.x, n_msg.vector.y, n_msg.vector.z};
    double dot_prod = dot_product(down, n_map2);

    // Check if n represents wall
    if(fabs(dot_prod) < wall_thresh){
        // Update normal vector to [0,0,-1] in map2 frame for next iteration geometrical subcloud
        if(last_iteration && subcloud == GEOMETRICAL){
            try {
                // Listen to tf tree for transformation
                t = tf_buffer.lookupTransform("pandar_frame", "map2", ros::Time(0));
            } catch(tf2::TransformException& ex){
                ROS_ERROR("Failed to listen to tf tree!\n\n");
                return false;
            }
            // Create "down" Vector n in map2 frame
            geometry_msgs::Vector3Stamped n_map;
            n_map.vector.x = 0.0;
            n_map.vector.y = 0.0;
            n_map.vector.z = -1.0;

            tf2::doTransform(n_map, n_pandar, t);
            n[0] = n_pandar.vector.x;
            n[1] = n_pandar.vector.y;
            n[2] = n_pandar.vector.z;
            if(!quiet) ROS_ERROR("Resetting n!");
        }
        return false;
    }

    // The dot product = positive if the angle between both vectors is smaller than 90 degrees, and negative otherwise.
    if(dot_prod < 0){
        n[0] = -n[0];
        n[1] = -n[1];
        n[2] = -n[2];
        n_msg.vector.x = -n_msg.vector.x;
        n_msg.vector.y = -n_msg.vector.y;
        n_msg.vector.z = -n_msg.vector.z;
    }

    // If successfully found plane then caluculate and print inclination
    if(!quiet){
        n_map2 = {n_msg.vector.x, n_msg.vector.y, n_msg.vector.z};
        dot_prod = dot_product(down, n_map2);
        double inclination = acos(dot_prod) * 180 / M_PI;
        ROS_INFO("Inclination of the plane: %.5f", inclination);
    }

    return true;
}

void GroundFinder::scan_callback(const sensor_msgs::PointCloud2ConstPtr& msg){
    // Convert PointCloud2 (sensor msg) to PointCloud (pcl)
    pcl::PointCloud<PointType>::Ptr cur_scan(new pcl::PointCloud<PointType>);
    pcl::fromROSMsg(*msg, *cur_scan);

    // Total duration [ns]
    int64_t duration_total = 0;

    // Calculation of query point for creation of subcloud (used for both approaches)
    PointType query_point = {0.0, 0.0, 0.0};
    query_point.x = n[0] * radius_sphere;
    query_point.y = n[1] * radius_sphere;
    query_point.z = n[2] * radius_sphere;

    // ---------------------- Preproccessing of scan ----------------------

    if(filtering == GEOMETRICAL && subcloud == GEOMETRICAL){
        // If both approaches = geo, then only looping once through entire point cloud (faster)
        duration_total += downsample(cur_scan);
        duration_total += filter_create_subcloud_geo(cur_scan, query_point);
    } else {
        // A) Filter cloud: Take out reflections from sphere + hands & downsample or not (filtering == NONE)
        int64_t duration_filter = 0;
        if(filtering == GEOMETRICAL){
            duration_filter += downsample(cur_scan);
            duration_filter += filter_cloud_geo(cur_scan);
        } else if(filtering == KD_TREE){
            duration_filter += downsample(cur_scan);
            duration_filter += filter_cloud_kdt(cur_scan);
        } else {
            if(write2file) csv << "0,0,0,0,";
        }

        // B) Create subcloud
        int64_t duration_subcloud = 0;
        if(subcloud == GEOMETRICAL){
            duration_subcloud = create_subcloud_geo(cur_scan, query_point);
        } else if(subcloud == KD_TREE){
            duration_subcloud = create_subcloud_kdt(cur_scan, k, query_point);
        }
        // Total preprocessing time
        duration_total = duration_filter + duration_subcloud;
    }
    if(!quiet) ROS_WARN("Total preprocessing time: %0.6f ms", float(duration_total) / 1e3);
    if(write2file) csv << duration_total << ",";

    // Publish (filtered) subcloud
    sensor_msgs::PointCloud2::Ptr sub_cloud_msg(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(*cur_scan, *sub_cloud_msg);
    sub_cloud_msg->header.stamp = msg->header.stamp;
    sub_cloud_msg->header.frame_id = msg->header.frame_id;
    pub_subcloud.publish(*sub_cloud_msg);


    // ---------------------- Plane segmentation ----------------------

    geometry_msgs::Vector3Stamped n_msg;
    n_msg.header.frame_id = "map2";
    n_msg.header.stamp = msg->header.stamp;
    // Determine normal vector of ground in map2 frame incl. ensuring it represents ground and points into ground
    auto duration_plane = determine_n_ground_plane(cur_scan, plane_alg, n_msg);
    // Write to file
    if(write2file) csv << duration_plane << ",";

    if(duration_plane < 0){
        ROS_ERROR("Plane segmentation fault!\n");
        count_fail++;
        // Write to file
        if(write2file) csv << "-1000,-1,-1,-1," << plane_counter << "\n";
        return;
    }

    // Total time
    duration_total += duration_plane;
    if(!quiet) ROS_WARN("Total time: %0.6f ms; fails: %d\n", float(duration_total) / 1e3, count_fail);
    // Write to file
    if(write2file) csv << duration_total << "," << n_msg.vector.x << "," << n_msg.vector.y << "," << n_msg.vector.z << ",";
    // Plane counter
    if(write2file) csv << plane_counter << "\n";

    // Publish normal vector
    pub_n.publish(n_msg);

    // Update normal vector n_marker
    n_marker.header.frame_id = msg->header.frame_id;
    n_marker.header.stamp = msg->header.stamp;
    geometry_msgs::Point start, end;
    start.x = query_point.x;
    start.y = query_point.y;
    start.z = query_point.z;
    end.x = (start.x + n[0])/3.0;
    end.y = (start.y + n[1])/3.0;
    end.z = (start.z + n[2])/3.0;
    n_marker.points = {start, end};
    // Publish n_marker
    pub_vis_n.publish(n_marker);
}

void GroundFinder::scan_callback_count(const std_msgs::EmptyConstPtr& msg){
    // if(!quiet){
    ROS_WARN("MSG on 'plane' received -> Indicatiing new plane! Increasing counter.");
    // }
    plane_counter++;
}
