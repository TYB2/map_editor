#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include <eigen3/Eigen/Core>
// #include <pcl-1.8/pcl/io/ply_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <map>

using namespace std;

class Mapping{
private:
    /*------------------------私有变量--------------------------*/
    struct MapStruct{
        unsigned char map_state;                // 0代表free状态，1代表occupancy状态，2代表膨胀的occpancy，1能覆盖2，2不能覆盖1，所以每次赋值2的时候都需要判断该点是否为1
        int count;                              // 类似智能指针，对膨胀区域计数，每当被膨胀一次count加1，只有当记数为0，该点才会被清除
                                                // 在膨胀时，如果该点为1，仍需要记数，如果该1点被清除，1退化为2；清除膨胀区域同理
    };

    int size_x;
    int size_y;
    int size_z;
    int offset_x, offset_y, offset_z;
    double map_resolusion;
    int expand_size;                            // 膨胀的尺寸，单位像素
    int occ_heght;                              // 障碍物的高度
    geometry_msgs::PoseStamped camera_pose;     // test
    Eigen::MatrixX3d camera_R;                  // test
    Eigen::Vector3d  camera_position;                // test

    MapStruct*** my_map;                        // 地图
    // map<vector<int>, int> occupancy_list;    // 障碍物序列，映射关系： <坐标> —— <编号>
    vector<vector<int>> occupancy_list;         // 障碍物序列，因为障碍物不多，这种更合理
    int id;                                     // 编号，从0开始，用过该编号后

    /*------------------------私有函数--------------------------*/
    // 
    /**
     * 更新障碍物序列，暂时先不实现
     * 
     * @param occ_pos       - 需要更新的障碍物坐标
    */
    // void updateOccupancyList(vector<int> occ_pos);

    /**
     * 判断点是否超出地图边界
     * 
     * @param pt            - 待判断的点
     * 
     * @return              - true，超出地图边界；false，在地图边界内
    */
    bool judgeOutOfMap(vector<int> pt);

    /**
     * 更新地图，主要是根据新出现的障碍物更新地图
     * 
     * @param occ_pos       - 需要更新的障碍物坐标
    */
    void updateMap(vector<int> occ_pos);

    /**
     * 清除障碍物
     * 
     * @param occ_pos       - 需要清除的障碍物坐标
    */
    void clearOccpancyInMap(vector<int> occ_pos);

    /**
     * 世界坐标转地图坐标
     * 
     * @param world_pos     - 世界坐标系下的坐标
     * 
     * @return              - 返回地图坐标系下的坐标
    */
    Eigen::Vector3i worldIndex2MapIndex(const Eigen::Vector3d& world_pos);

    /**
     * 对地图中的障碍物做膨胀，这里膨胀成矩形，会更容易维护一些
     * 最好膨胀无人机半径的1.5倍，这样就可以只检查无人机的坐标就能检验碰撞了
     * 
     * @param occ_pos       - 需要膨胀的障碍物坐标
    */
    void expandOccpancy(vector<int> occ_pos);

    /**
     * 计算距离，要求输入坐标为2维度
     * 
     * @param p1            - 坐标1
     * @param p2            - 坐标2
     * @return              - 两坐标的距离
    */
    double calDistance(vector<int> p1, vector<int> p2);

public:
    /*------------------------公有函数--------------------------*/
    /**
     * @param size_x_           - 地图沿x轴最大长度
     * @param size_y_           - 地图沿y轴最大长度
     * @param size_z_           - 地图沿z轴最大长度
     * @param map_resolusion_   - 地图分辨率
     * @param expand_size_      - 膨胀尺寸，单位m，推荐无人机半径的1.5倍
    */
    Mapping(int size_x_, int size_y_, int size_z_, double map_resolusion_, double expand_size_);
    ~Mapping();
    
    // 通过读取地图文件来初始画地图（暂时先不实现）
    // void initMapFromFile(const string& filename); 

    /**
     * 通过设置地图边界来初始化地图
     * 
     * @param map_size_x    - 地图宽度，沿x轴的长度，单位m
     * @param map_size_y    - 地图长度，沿y轴的长度，单位m
     * @param map_size_z    - 地图高度，沿z轴的长度，单位m
    */
    void initMap(const double& map_size_x, const double& map_size_y, const double& map_size_z);

    /**
     * 从.ply文件中读取点云并转换为当前地图格式
     * 
     * @param cloud_point_dir   - 点云文件的路径
    */
    void initMapFromCloudPoint(string cloud_point_dir);

    /**
     * 接收坐标
     * 
     * @param pos           - 接受的坐标
    */
    void acceptPosition(const geometry_msgs::PoseStamped& pos);

    /**
     * 获取地图中碰撞信息
     * 
     * @param world_pos     - 世界坐标系下坐标
     * 
     * @return              - 得到那一个点的占据状态
    */
    unsigned char getPositionState(const Eigen::Vector3d& world_pos);  

    /**
     * 获取地图中障碍物区域，膨胀区域和障碍物区域分开去除
     * 
     * @param occ_pts       - 存放障碍物区域点的数组
     * @param occ_exp_pts   - 存放障碍物膨胀区域点的数组
    */
    void getOccPts(sensor_msgs::PointCloud& occ_pts, sensor_msgs::PointCloud& occ_exp_pts);  

    void testRegisterOccpancy(const sensor_msgs::PointCloud& occ_pts);
};

inline double Mapping::calDistance(vector<int> p1, vector<int> p2){
    if(p1.size() != 2 || p2.size() != 2){
        cout << "error in Mapping::calDistance(): p1.size() != 2 || p2.size() != 2" << endl;
        exit(-1);
    }

    return sqrt(pow(p1[0] - p2[0], 2) + pow(p1[1] - p2[1], 2));
}