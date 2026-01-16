
#include "include/STDesc.h"
#include "include/trianglarMatching.h"

#include <livox_ros_driver/CustomMsg.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <signal.h>

using namespace std;

int findPoseIndexUsingTime(std::vector<double> &time_list, double &time) {
    double time_inc = 10000000000;
    int min_index = -1;
    for (size_t i = 0; i < time_list.size(); i++) {
        if (fabs(time_list[i] - time) < time_inc) {
            time_inc = fabs(time_list[i] - time);
            min_index = i;
        }
    }
    return min_index;
}

void livoMsg_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg, pcl::PointCloud<pcl::PointXYZI>& pl_surf){

    pcl::PointCloud<pcl::PointXYZI> pl_full;
    double t1 = omp_get_wtime();
    int plsize = msg->point_num;
    // cout<<"plsie: "<<plsize<<endl;

    pl_surf.reserve(plsize);
    pl_full.reserve(plsize);
    uint valid_num = 0;

    for(uint i=1; i<plsize; i++){
        if((msg->points[i].line < 6) &&
           ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
        {
            valid_num ++;

            pl_full[i].x = msg->points[i].x;
            pl_full[i].y = msg->points[i].y;
            pl_full[i].z = msg->points[i].z;
            pl_full[i].intensity = msg->points[i].reflectivity;
//                pl_full[i].curvature = msg->points[i].offset_time / float(1000000); // use curvature as time of each laser points, curvature unit: ms

            if((abs(pl_full[i].x - pl_full[i-1].x) > 1e-7)
               || (abs(pl_full[i].y - pl_full[i-1].y) > 1e-7)
               || (abs(pl_full[i].z - pl_full[i-1].z) > 1e-7)
                  && (pl_full[i].x * pl_full[i].x + pl_full[i].y * pl_full[i].y + pl_full[i].z * pl_full[i].z > 4))
                pl_surf.push_back(pl_full[i]);
        }
    }
}

void mySigintHandler(int sig){
    ros::shutdown();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "demo_livox");
    ros::NodeHandle nh;
    std::string bag_path = "";
    std::string pose_path = "";
    bool localizeMode = true;
    nh.param<std::string>("bag_path", bag_path, "");
    nh.param<std::string>("pose_path", pose_path, "");
    nh.param<bool>("localizeMode", localizeMode, true);

    ConfigSetting config_setting;
    read_parameters(nh, config_setting);

//    if (localizeMode){
//        ros::Subscriber sub_full = nh.subscribe<sensor_msgs::PointCloud2>("fullCloudTopic", 100, full_handler);
//        ros::Subscriber sub_odom = nh.subscribe<nav_msgs::Odometry>("poseTopic", 100, odom_handler);
//    }
    ros::Publisher pubOdomAftMapped =
            nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 10);
    ros::Publisher pubCureentCloud =
            nh.advertise<sensor_msgs::PointCloud2>("/cloud_current", 100);
    ros::Publisher pubCurrentCorner =
            nh.advertise<sensor_msgs::PointCloud2>("/cloud_key_points", 100);
    ros::Publisher pubMatchedCloud =
            nh.advertise<sensor_msgs::PointCloud2>("/cloud_matched", 100);
    ros::Publisher pubMatchedCorner =
            nh.advertise<sensor_msgs::PointCloud2>("/cloud_matched_key_points", 100);
    ros::Publisher pubSTD =
            nh.advertise<visualization_msgs::MarkerArray>("descriptor_line", 10);

    ros::Rate loop(60);
    ros::Rate slow_loop(0.1);
    std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> poses_vec;
    std::vector<double> times_vec;
    load_pose_with_time(pose_path, poses_vec, times_vec);
    std::cout << "Successfully load pose with number: " << poses_vec.size()
              << std::endl;

    STDescManager *std_manager = new STDescManager(config_setting);

    size_t cloudInd = 0;
    size_t keyCloudInd = 0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(
            new pcl::PointCloud<pcl::PointXYZI>());

    std::vector<double> descriptor_time;
    std::vector<double> querying_time;
    std::vector<double> update_time;
    int triggle_loop_num = 0;

    std::fstream file_;
    file_.open(bag_path, std::ios::in);
    if (!file_) {
        std::cout << "File " << bag_path << " does not exit" << std::endl;
    }
    ROS_INFO("Start to load the rosbag %s", bag_path.c_str());
    rosbag::Bag bag;
    try {
        bag.open(bag_path, rosbag::bagmode::Read);
    } catch (rosbag::BagException e) {
        ROS_ERROR_STREAM("LOADING BAG FAILED: " << e.what());
    }
    std::vector<std::string> types;
    types.push_back(std::string("sensor_msgs/PointCloud2"));
    rosbag::View view(bag, rosbag::TypeQuery(types));

    while (ros::ok()) {
//        if (!localizeMode)
        BOOST_FOREACH (rosbag::MessageInstance const m, view) {
                        sensor_msgs::PointCloud2::ConstPtr cloud_ptr =
                                m.instantiate<sensor_msgs::PointCloud2>();
                        if (cloud_ptr != NULL) {
                            double laser_time = cloud_ptr->header.stamp.toSec();
                            pcl::PCLPointCloud2 pcl_pc;
                            pcl_conversions::toPCL(*cloud_ptr, pcl_pc);
                            pcl::PointCloud<pcl::PointXYZI> cloud;
                            pcl::fromPCLPointCloud2(pcl_pc, cloud);
                            int pose_index = findPoseIndexUsingTime(times_vec, laser_time);
                            if (abs(laser_time - times_vec[pose_index]) > 0.2) {
//                                    cout << "Skip scan ..." << std::to_string(laser_time) << endl;
                                continue;
                            }

                            Eigen::Vector3d translation = poses_vec[pose_index].first;
                            Eigen::Matrix3d rotation = poses_vec[pose_index].second;
                            for (size_t i = 0; i < cloud.size(); i++) {
                                Eigen::Vector3d pv = point2vec(cloud.points[i]);
                                pv = rotation * pv + translation;
                                cloud.points[i] = vec2point(pv);
                            }
                            down_sampling_voxel(cloud, config_setting.ds_size_);
                            for (auto pv : cloud.points)
                                temp_cloud->points.push_back(pv);

                            // check if keyframe
                            if (cloudInd % config_setting.sub_frame_num_ == 0 && cloudInd != 0) {
                                std::cout << "Key Frame id:" << keyCloudInd
                                          << ", cloud size: " << temp_cloud->size() << std::endl;
                                // step1. Descriptor Extraction
                                auto t_descriptor_begin = std::chrono::high_resolution_clock::now();
                                std::vector<STDesc> stds_vec;
                                std_manager->GenerateSTDescs(temp_cloud, stds_vec);
                                auto t_descriptor_end = std::chrono::high_resolution_clock::now();
                                descriptor_time.push_back(
                                        time_inc(t_descriptor_end, t_descriptor_begin));
                                // step2. Searching Loop
                                auto t_query_begin = std::chrono::high_resolution_clock::now();
                                std::pair<int, double> search_result(-1, 0);
                                std::pair<Eigen::Vector3d, Eigen::Matrix3d> loop_transform;
                                loop_transform.first << 0, 0, 0;
                                loop_transform.second = Eigen::Matrix3d::Identity();
                                std::vector<std::pair<STDesc, STDesc>> loop_std_pair;
                                if (keyCloudInd > config_setting.skip_near_num_)
                                    std_manager->SearchLoop(stds_vec, search_result, loop_transform,
                                                            loop_std_pair);

                                if (search_result.first > 0)
                                    std::cout << "[Loop Detection] trigger loop: " << keyCloudInd
                                              << "--" << search_result.first
                                              << ", score:" << search_result.second << std::endl;

                                auto t_query_end = std::chrono::high_resolution_clock::now();
                                querying_time.push_back(time_inc(t_query_end, t_query_begin));

                                // step3. Add descriptors to the database
                                auto t_map_update_begin = std::chrono::high_resolution_clock::now();
                                std_manager->AddSTDescs(stds_vec);
                                auto t_map_update_end = std::chrono::high_resolution_clock::now();
                                update_time.push_back(time_inc(t_map_update_end, t_map_update_begin));
                                std::cout << "[Time] descriptor extraction: "
                                          << time_inc(t_descriptor_end, t_descriptor_begin) << "ms, "
                                          << "query: " << time_inc(t_query_end, t_query_begin)
                                          << "ms, "
                                          << "update map:"
                                          << time_inc(t_map_update_end, t_map_update_begin) << "ms"
                                          << std::endl;
                                std::cout << std::endl;

                                pcl::PointCloud<pcl::PointXYZI> save_key_cloud;
                                save_key_cloud = *temp_cloud;

                                std_manager->key_cloud_vec_.push_back(save_key_cloud.makeShared());

                                // publish
                                sensor_msgs::PointCloud2 pub_cloud;
                                pcl::toROSMsg(*temp_cloud, pub_cloud);
                                pub_cloud.header.frame_id = "camera_init";
                                pubCureentCloud.publish(pub_cloud);
                                pcl::toROSMsg(*std_manager->corner_cloud_vec_.back(), pub_cloud);
                                pub_cloud.header.frame_id = "camera_init";
                                pubCurrentCorner.publish(pub_cloud);

                                if (search_result.first > 0) {
                                    triggle_loop_num++;
                                    pcl::toROSMsg(*std_manager->key_cloud_vec_[search_result.first],
                                                  pub_cloud);
                                    pub_cloud.header.frame_id = "camera_init";
                                    pubMatchedCloud.publish(pub_cloud);
                                    slow_loop.sleep();
                                    pcl::toROSMsg(*std_manager->corner_cloud_vec_[search_result.first],
                                                  pub_cloud);
                                    pub_cloud.header.frame_id = "camera_init";
                                    pubMatchedCorner.publish(pub_cloud);
                                    publish_std_pairs(loop_std_pair, pubSTD);
                                    slow_loop.sleep();

                                    Cloud3D srcTriCloud, tgtTriCloud;
                                    pcl::copyPointCloud<pcl::PointXYZI, Point3D>(*std_manager->key_cloud_vec_.back(),
                                                                                 srcTriCloud);
                                    pcl::copyPointCloud<pcl::PointXYZI, Point3D>(*std_manager->key_cloud_vec_[search_result.first],
                                                                                 tgtTriCloud);
                                    Eigen::Matrix4f relaTrans = Eigen::Matrix4f::Identity();
                                    TriangleMatching triangleMatcher;
                                    triangleMatcher.setPairwiseStemPositions(srcTriCloud.makeShared(),
                                                                             tgtTriCloud.makeShared());
                                    triangleMatcher.estimateTransformation(relaTrans);
                                    cout << relaTrans.matrix() << endl;

                                    int com;
                                    cin >> com;  // wait for input
                                }
                                temp_cloud->clear();
                                keyCloudInd++;
                                loop.sleep();
                            }

                            nav_msgs::Odometry odom;
                            odom.header.frame_id = "camera_init";
                            odom.pose.pose.position.x = translation[0];
                            odom.pose.pose.position.y = translation[1];
                            odom.pose.pose.position.z = translation[2];
                            Eigen::Quaterniond q(rotation);
                            odom.pose.pose.orientation.w = q.w();
                            odom.pose.pose.orientation.x = q.x();
                            odom.pose.pose.orientation.y = q.y();
                            odom.pose.pose.orientation.z = q.z();
                            pubOdomAftMapped.publish(odom);
                            loop.sleep();
                            cloudInd++;
                        }
                    }

        double mean_descriptor_time =
                std::accumulate(descriptor_time.begin(), descriptor_time.end(), 0) *
                1.0 / descriptor_time.size();
        double mean_query_time =
                std::accumulate(querying_time.begin(), querying_time.end(), 0) * 1.0 /
                querying_time.size();
        double mean_update_time =
                std::accumulate(update_time.begin(), update_time.end(), 0) * 1.0 /
                update_time.size();
        std::cout << "Total key frame number:" << keyCloudInd
                  << ", loop number:" << triggle_loop_num << std::endl;
        std::cout << "Mean time for descriptor extraction: " << mean_descriptor_time
                  << "ms, query: " << mean_query_time
                  << "ms, update: " << mean_update_time << "ms, total: "
                  << mean_descriptor_time + mean_query_time + mean_update_time
                  << "ms" << std::endl;


        ///region # locate in another
        if(localizeMode){

            pose_path = "/home/cyz/workspace/catkin_main/src/FAST_LIO-main/PCD/odomPoses-mid360-wuzhou.txt";
            bag_path = "/home/cyz/下载/dockerCodes/STD-master/livox_dataset/ugv-mid-undistorScan-wuzhou.bag";

            poses_vec.clear();  times_vec.clear();
            load_pose_with_time(pose_path, poses_vec, times_vec);
            std::cout << "Successfully load pose with number: " << poses_vec.size() << std::endl;
            int lastKeyFrameSize = keyCloudInd;
            int margRatio = 0.6 * config_setting.sub_frame_num_;
            pcl::PointCloud<pcl::PointXYZI>::Ptr marg_cloud(new pcl::PointCloud<pcl::PointXYZI>());

            std::fstream file2;
            file2.open(bag_path, std::ios::in);
            if (!file2)
                std::cout << "File " << bag_path << " does not exit" << std::endl;

            ROS_INFO("Start to load the rosbag %s", bag_path.c_str());
            rosbag::Bag bag2;
            try {
                bag2.open(bag_path, rosbag::bagmode::Read);
            } catch (rosbag::BagException e) {
                ROS_ERROR_STREAM("LOADING BAG FAILED: " << e.what());
            }
            rosbag::View view2(bag2, rosbag::TypeQuery(types));
            cloudInd = 0;
            std_manager->config_setting_.icp_threshold_ = 0.03;
//            std_manager->config_setting_.skip_near_num_ = 200;
//            config_setting.sub_frame_num_ = 40;

            BOOST_FOREACH(rosbag::MessageInstance
                                  const m, view2) {
                            sensor_msgs::PointCloud2::ConstPtr cloud_ptr =
                                    m.instantiate<sensor_msgs::PointCloud2>();
                            if (cloud_ptr == nullptr) continue;

                            double laser_time = cloud_ptr->header.stamp.toSec();
                            pcl::PCLPointCloud2 pcl_pc;
                            pcl_conversions::toPCL(*cloud_ptr, pcl_pc);
                            pcl::PointCloud<pcl::PointXYZI> cloud;
                            pcl::fromPCLPointCloud2(pcl_pc, cloud);
                            int pose_index = findPoseIndexUsingTime(times_vec, laser_time);
                            if (abs(laser_time - times_vec[pose_index]) > 0.2) {
//                                cout << "Skip scan ..." << std::to_string(laser_time) << endl;
                                continue;
                            }

                            Eigen::Vector3d translation = poses_vec[pose_index].first;
                            Eigen::Matrix3d rotation = poses_vec[pose_index].second;
                            for (size_t i = 0; i < cloud.size(); i++) {
                                Eigen::Vector3d pv = point2vec(cloud.points[i]);
                                pv = rotation * pv + translation;
                                cloud.points[i] = vec2point(pv);
                            }
                            down_sampling_voxel(cloud, config_setting.ds_size_);
                            for (auto pv : cloud.points)
                                temp_cloud->points.push_back(pv);

                            // check if keyframe
                            if (cloudInd % config_setting.sub_frame_num_ == 0 && cloudInd != 0) {
                                std::cout << "Key Frame id:" << keyCloudInd
                                          << ", cloud size: " << temp_cloud->size() << std::endl;
                                // step1. Descriptor Extraction
                                auto t_descriptor_begin = std::chrono::high_resolution_clock::now();
                                std::vector<STDesc> stds_vec;
                                std_manager->GenerateSTDescs(temp_cloud, stds_vec);
                                auto t_descriptor_end = std::chrono::high_resolution_clock::now();
                                descriptor_time.push_back(
                                        time_inc(t_descriptor_end, t_descriptor_begin));
                                // step2. Searching Loop
                                auto t_query_begin = std::chrono::high_resolution_clock::now();
                                std::pair<int, double> search_result(-1, 0);
                                std::pair<Eigen::Vector3d, Eigen::Matrix3d> loop_transform;
                                loop_transform.first << 0, 0, 0;
                                loop_transform.second = Eigen::Matrix3d::Identity();
                                std::vector<std::pair<STDesc, STDesc>> loop_std_pair;
                                if (keyCloudInd > config_setting.skip_near_num_)
                                    std_manager->SearchLoop(stds_vec, search_result, loop_transform,
                                                            loop_std_pair);

                                if (search_result.first > 0)
                                    std::cout << "[Loop Detection] trigger loop: " << keyCloudInd
                                              << "--" << search_result.first
                                              << ", score:" << search_result.second << std::endl;

                                auto t_query_end = std::chrono::high_resolution_clock::now();
                                querying_time.push_back(time_inc(t_query_end, t_query_begin));

                                // step3. Add descriptors to the database
                                auto t_map_update_begin = std::chrono::high_resolution_clock::now();
                                std_manager->AddSTDescs(stds_vec);
                                auto t_map_update_end = std::chrono::high_resolution_clock::now();
                                update_time.push_back(time_inc(t_map_update_end, t_map_update_begin));
                                std::cout << "[Time] descriptor extraction: "
                                          << time_inc(t_descriptor_end, t_descriptor_begin) << "ms, "
                                          << "query: " << time_inc(t_query_end, t_query_begin)
                                          << "ms, "
                                          << "update map:"
                                          << time_inc(t_map_update_end, t_map_update_begin) << "ms"
                                          << std::endl;
                                std::cout << std::endl;

                                pcl::PointCloud<pcl::PointXYZI> save_key_cloud;
                                save_key_cloud = *temp_cloud;

                                std_manager->key_cloud_vec_.push_back(save_key_cloud.makeShared());

                                // publish
                                sensor_msgs::PointCloud2 pub_cloud;
                                pcl::toROSMsg(*temp_cloud, pub_cloud);
                                pub_cloud.header.frame_id = "camera_init";
                                pubCureentCloud.publish(pub_cloud);
                                pcl::toROSMsg(*std_manager->corner_cloud_vec_.back(), pub_cloud);
                                pub_cloud.header.frame_id = "camera_init";
                                pubCurrentCorner.publish(pub_cloud);

                                if (search_result.first > 0 && search_result.first < lastKeyFrameSize) {
                                    triggle_loop_num++;
                                    pcl::toROSMsg(*std_manager->key_cloud_vec_[search_result.first],
                                                  pub_cloud);
                                    pub_cloud.header.frame_id = "camera_init";
                                    pubMatchedCloud.publish(pub_cloud);
                                    slow_loop.sleep();
                                    pcl::toROSMsg(*std_manager->corner_cloud_vec_[search_result.first],
                                                  pub_cloud);
                                    pub_cloud.header.frame_id = "camera_init";
                                    pubMatchedCorner.publish(pub_cloud);
                                    publish_std_pairs(loop_std_pair, pubSTD);
                                    slow_loop.sleep();

                                    cout << "[ Debug ] triangle matching ... " << endl;
                                    Cloud3D srcTriCloud, tgtTriCloud;
                                    pcl::copyPointCloud<pcl::PointXYZINormal, Point3D>
                                            (*std_manager->corner_cloud_vec_.back(),srcTriCloud);
                                    pcl::copyPointCloud<pcl::PointXYZINormal, Point3D>
                                            (*std_manager->corner_cloud_vec_[search_result.first],tgtTriCloud);

                                    Eigen::Matrix4f relaTrans = Eigen::Matrix4f::Identity();
                                    TriangleMatching triangleMatcher;
                                    triangleMatcher.setPairwiseStemPositions(srcTriCloud.makeShared(),
                                                                             tgtTriCloud.makeShared());

                                    triangleMatcher.estimateTransformation(relaTrans);
                                    cout << relaTrans.matrix() << endl;

                                    int cnt;
                                    while(cnt != 0)
                                        cin >> cnt;

                                    pcl::io::savePCDFileBinaryCompressed("/home/cyz/下载/dockerCodes/STD-master/livox_dataset/srcCloud"
                                                                         +to_string(1)+".pcd", *std_manager->key_cloud_vec_.back());
//                                                                         +to_string(keyCloudInd)+".pcd", *std_manager->key_cloud_vec_.back());
                                    pcl::io::savePCDFileBinaryCompressed("/home/cyz/下载/dockerCodes/STD-master/livox_dataset/tgtCloud"
                                                                         +to_string(1)+".pcd", *std_manager->key_cloud_vec_[search_result.first]);
//                                                                         +to_string(search_result.first)+".pcd", *std_manager->key_cloud_vec_[search_result.first]);
                                }
//                                temp_cloud->clear();
                                temp_cloud->points.swap(marg_cloud->points);
                                marg_cloud->clear();

                                keyCloudInd++;
                                loop.sleep();

                            }else if(cloudInd % config_setting.sub_frame_num_ > margRatio){
                                for (auto pv : cloud.points)
                                    marg_cloud->points.push_back(pv);
                            }

                            nav_msgs::Odometry odom;
                            odom.header.frame_id = "camera_init";
                            odom.pose.pose.position.x = translation[0];
                            odom.pose.pose.position.y = translation[1];
                            odom.pose.pose.position.z = translation[2];
                            Eigen::Quaterniond q(rotation);
                            odom.pose.pose.orientation.w = q.w();
                            odom.pose.pose.orientation.x = q.x();
                            odom.pose.pose.orientation.y = q.y();
                            odom.pose.pose.orientation.z = q.z();
                            pubOdomAftMapped.publish(odom);
                            loop.sleep();
                            cloudInd++;
                        }
        }
        break;
        // endregion

    }

    signal(SIGINT, mySigintHandler);
    return 0;
}