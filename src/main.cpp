#include "mapping.h"

Mapping* my_map_editor;
double size_x, size_y, size_z;
double wall_size_x, wall_size_y, wall_size_z;
double map_resolusion, expand_size;
string cloud_point_filename;

ros::Publisher occupancy_pub;
ros::Publisher occupancy_expand_pub;

sensor_msgs::PointCloud occ_pts;
sensor_msgs::PointCloud occ_exp_pts;

geometry_msgs::PoseStamped accept_pose;

// void get_pt_cb(const geometry_msgs::PoseStamped::ConstPtr& pt){
//     my_map_editor->acceptPosition(*pt);
// }

void get_pt_cb(const geometry_msgs::PointStamped::ConstPtr& pt){

    accept_pose.pose.position.x = pt->point.x;
    accept_pose.pose.position.y = pt->point.y;
    // my_map_editor->acceptPosition(accept_pose);

    sensor_msgs::PointCloud occ_line;

    geometry_msgs::Point32 start_pt;
    start_pt.x = 1.0;
    start_pt.y = 1.0;
    start_pt.z = 1.0;
    geometry_msgs::Point32 end_pt;
    end_pt.x = 1.0;
    end_pt.y = 1.0;
    end_pt.z = 0.5;
    occ_line.points.push_back(start_pt);
    occ_line.points.push_back(end_pt);
    my_map_editor->testRegisterOccpancy(occ_line);
}

void publish_callback(const ros::TimerEvent& e){
    // cout << "getOccPts" << endl;
    my_map_editor->getOccPts(occ_pts, occ_exp_pts);

    // publish map
    // cout << "start publish" << endl;
    occupancy_pub.publish(occ_pts);
    occupancy_expand_pub.publish(occ_exp_pts);
}

void init(){
    cout << "new Mapping" <<endl;
    my_map_editor = new Mapping(int(size_x / map_resolusion), 
                                int(size_y / map_resolusion), 
                                int(size_z / map_resolusion), 
                                map_resolusion, 
                                expand_size);

    cout << "init map" << endl;
    // my_map_editor->initMap(wall_size_x, wall_size_y, wall_size_z);
    // my_map_editor->initMapFromCloudPoint(cloud_point_filename);

    occ_pts.header.frame_id = "map";
    occ_exp_pts.header.frame_id = "map";

    cout << "finish init" << endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_editor");
    ros::NodeHandle node("~");
    ros::NodeHandle nh;

    string accept_pt_topic;
    node.param("size_x", size_x, 5.);
    node.param("size_y", size_y, 5.);
    node.param("size_z", size_z, 3.);
    node.param("wall_size_x", wall_size_x, 4.);
    node.param("wall_size_y", wall_size_y, 4.);
    node.param("wall_size_z", wall_size_z, 1.9);
    node.param("map_resolusion", map_resolusion, 0.1);
    node.param("expand_size", expand_size, 0.4);
    node.param("accept_pt_topic", accept_pt_topic, string("/clicked_point"));
    node.param("cloud_point_filename", cloud_point_filename, string("null"));

    if(wall_size_x > size_x || wall_size_x > size_y || wall_size_z > size_z){
        cout << "size of wall can not larger than size of map" << endl;
        exit(-1);
    }

    if(cloud_point_filename.compare("null") == 0){
        cout << "cloud_point_filename can not be empty" << endl;
        exit(-1);
    }

    init();

    // 接受坐标
    ros::Subscriber get_pt_sub = nh.subscribe(accept_pt_topic, 1, get_pt_cb);

    // 可视化
    occupancy_pub = nh.advertise<sensor_msgs::PointCloud>("visualize_pointcloud", 1, true);
    occupancy_expand_pub = nh.advertise<sensor_msgs::PointCloud>("visualize_expand_pointcloud", 1, true);
    ros::Timer publish_timer = nh.createTimer(ros::Duration(0.1), publish_callback); // 发布频率为10Hz
    
    ros::spin();

    delete my_map_editor;

    return 0;
}
