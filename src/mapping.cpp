#include "mapping.h"

Mapping::Mapping(   int size_x_, 
                    int size_y_, 
                    int size_z_,
                    double map_resolusion_,
                    double expand_size_
                ): size_x(size_x_), size_y(size_y_), size_z(size_z_), map_resolusion(map_resolusion_){

    // 为地图分配空间 并 赋值
    my_map = new MapStruct**[size_x];
    for(int i = 0; i < size_x; i++){

        my_map[i] = new MapStruct*[size_y];
        for(int j = 0; j < size_y; j++){

            my_map[i][j] = new MapStruct[size_z];
            for(int k = 0; k < size_z; k++){

                my_map[i][j][k].map_state = 0;
                my_map[i][j][k].count = 0;
            }
        }
    }

    // init
    expand_size = (int)(expand_size_ / map_resolusion);
    id = 0;
    occ_heght = min(int(1.5 / map_resolusion), size_z);
    offset_x = size_x / 2;
    offset_y = size_y / 2;
}

Mapping::~Mapping(){

    // 删除地图空间
    for(int i = 0; i < size_x; i++){
        for(int j = 0; j < size_y; j++){
            delete[] my_map[i][j];
        }

        delete[] my_map[i];
    }

    delete[] my_map;
}

void Mapping::initMap(const double& map_size_x, const double& map_size_y, const double& map_size_z){
    // 设置围墙
    int min_wall_x = -(map_size_x / 2) / map_resolusion + offset_x;
    int max_wall_x =  (map_size_x / 2) / map_resolusion + offset_x;
    int min_wall_y = -(map_size_y / 2) / map_resolusion + offset_y;
    int max_wall_y =  (map_size_y / 2) / map_resolusion + offset_y;
    int map_wall_z = min(size_z, (int)(map_size_z / map_resolusion));

    // 设置前围墙
    int y = max_wall_y;
    int x = min_wall_x;
    for(; x <= max_wall_x; ++x){
        for(int z = 0; z < map_wall_z; ++z){
            // 设置障碍物
            my_map[x][y][z].map_state = 1;

            // 设置膨胀，沿y轴向内膨胀
            for(int expand_y = y - expand_size; expand_y < y; ++expand_y){
                if(my_map[x][expand_y][z].map_state != 1){
                    my_map[x][expand_y][z].map_state = 2;
                }
                my_map[x][expand_y][z].count++;
            }
        }
    }

    // 设置后围墙
    y = min_wall_y;
    x = min_wall_x;
    for(; x <= max_wall_x; ++x){
        for(int z = 0; z < map_wall_z; ++z){
            // 设置障碍物
            my_map[x][y][z].map_state = 1;

            // 设置膨胀，沿y轴向内膨胀
            for(int expand_y = y + expand_size; expand_y > y; --expand_y){
                if(my_map[x][expand_y][z].map_state != 1){
                    my_map[x][expand_y][z].map_state = 2;
                }
                my_map[x][expand_y][z].count++;
            }
        }
    }

    // 设置左围墙
    y = min_wall_y;
    x = min_wall_x;
    for(; y <= max_wall_y; ++y){
        for(int z = 0; z < map_wall_z; ++z){
            // 设置障碍物
            my_map[x][y][z].map_state = 1;

            // 设置膨胀，沿x轴向内膨胀
            for(int expand_x = x + expand_size; expand_x > x; --expand_x){
                if(my_map[expand_x][y][z].map_state != 1){
                    my_map[expand_x][y][z].map_state = 2;
                }
                my_map[expand_x][y][z].count++;
            }
        }
    }

    // 设置右围墙
    y = min_wall_y;
    x = max_wall_x;
    for(; y <= max_wall_y; ++y){
        for(int z = 0; z < map_wall_z; ++z){
            // 设置障碍物
            my_map[x][y][z].map_state = 1;

            // 设置膨胀，沿x轴向内膨胀
            for(int expand_x = x - expand_size; expand_x < x; ++expand_x){
                if(my_map[expand_x][y][z].map_state != 1){
                    my_map[expand_x][y][z].map_state = 2;
                }
                my_map[expand_x][y][z].count++;
            }
        }
    }


    // 封顶（暂时不需要封顶）
}

Eigen::Vector3i Mapping::worldIndex2MapIndex(const Eigen::Vector3d& world_pos){
    // TODO 可能需要一定偏移量与世界坐标系对齐

    Eigen::Vector3i map_pos;

    map_pos.x() = world_pos.x() / map_resolusion + offset_x;
    map_pos.y() = world_pos.y() / map_resolusion + offset_y;
    map_pos.z() = world_pos.z() / map_resolusion;

    return map_pos;
}

bool Mapping::judgeOutOfMap(vector<int> pt){
    if( pt[0] < 0 || pt[1] < 0 ||
        pt[0] >= size_x || pt[1] >= size_y){
        return true;
    }

    return false;
}

void Mapping::acceptPosition(const geometry_msgs::PoseStamped& pos){
    // 接受坐标
    vector<int> accept_pos{int(pos.pose.position.x / map_resolusion) + offset_x, int(pos.pose.position.y / map_resolusion) + offset_y};

    // 需要判断接收到的点点是否超出地图边界
    if(judgeOutOfMap(accept_pos)){
        cout << "error position: out of map" << endl;
        return;
    }

    // 判断该位置是否存在障碍物，如不存在则更新；若存在则更新位置
    int update_occpancy_id = id;
    for(int i = 0; i < id; i++){

        double dis = calDistance(accept_pos, occupancy_list[i]); // TODO 这里可以优化，不需要开平方
        if(dis < 0.5){  // 太远则认为不是同一个障碍物，太近则认为是同一个障碍物，属于正常偏差范围，这两个参数是需要手动调整的 TODO 将这两个变量改为全局变量，可在外部调整
            return;
        }
        else if(dis < 2){
            // 清除原障碍物并更新序列
            clearOccpancyInMap(occupancy_list[i]);
            occupancy_list[i] = accept_pos;
            update_occpancy_id = i;
            break;
        }
    }

    // 根据坐标更新障碍物序列和地图
    if(update_occpancy_id == id){
        occupancy_list.push_back(accept_pos);
        id++;
    }
    updateMap(accept_pos);
}

void Mapping::clearOccpancyInMap(vector<int> occ_pos){
    
    // 清除1的区域，若该区域为其他障碍物的膨胀区域，则退化为2
    for(int z = 0; z < occ_heght; z++){
        if(my_map[occ_pos[0]][occ_pos[1]][z].count != 0){
            my_map[occ_pos[0]][occ_pos[1]][z].map_state = 2;
        }
        else{
            my_map[occ_pos[0]][occ_pos[1]][z].map_state = 0;
        }
    }

    // 清除膨胀区域，只有膨胀区域count==0才会被清除
    for(int z = 0; z < occ_heght; z++){

        int start_x = occ_pos[0] - expand_size;
        int end_x = occ_pos[0] + expand_size;
        for(int x = start_x; x <= end_x; x++){

            int start_y = occ_pos[1] - expand_size;
            int end_y = occ_pos[1] + expand_size;
            for(int y = start_y; y < end_y; y++){

                if(x == occ_pos[0] && y ==occ_pos[1]) continue;
                
                my_map[x][y][z].count--;
                if(my_map[x][y][z].map_state != 1){

                    if(my_map[x][y][z].count == 0){
                        
                        my_map[x][y][z].map_state = 0;
                    }
                }
            }
        }
    }

}

void Mapping::updateMap(vector<int> occ_pos){
    
    // 更新1区域
    for(int z = 0; z < occ_heght; z++){
        my_map[occ_pos[0]][occ_pos[1]][z].map_state = 1;
    }

    // 更新膨胀区域
    expandOccpancy(occ_pos);
}

void Mapping::expandOccpancy(vector<int> occ_pos){
    // 更新膨胀区域结点的信息
    for(int z = 0; z < occ_heght; z++){

        int start_x = max(occ_pos[0] - expand_size, 0);
        int end_x = min(occ_pos[0] + expand_size, size_x);
        for(int x = start_x; x <= end_x; x++){

            int start_y = max(occ_pos[1] - expand_size, 0);
            int end_y = min(occ_pos[1] + expand_size, size_y);
            for(int y = start_y; y < end_y; y++){

                if(x == occ_pos[0] && y ==occ_pos[1]) continue;
                
                my_map[x][y][z].count++;
                if(my_map[x][y][z].map_state != 1){
                    my_map[x][y][z].map_state = 2;
                }
            }
        }
    }

}

unsigned char Mapping::getPositionState(const Eigen::Vector3d& world_pos){
    // TODO 需要保证world_pos在地图范围内
    Eigen::Vector3i map_pos = worldIndex2MapIndex(world_pos);
    
    return my_map[map_pos[0]][map_pos[1]][map_pos[2]].map_state;
}

void Mapping::getOccPts(sensor_msgs::PointCloud& occ_pts, sensor_msgs::PointCloud& occ_exp_pts){
    const static int min_x = -size_x / 2 + offset_x;
    const static int max_x =  size_x / 2 + offset_x;
    const static int min_y = -size_y / 2 + offset_y;
    const static int max_y =  size_y / 2 + offset_y;
    const static int min_z =  0;
    const static int max_z =  size_z;

    occ_pts.points.clear();
    occ_exp_pts.points.clear();

    for(int x = min_x; x < max_x; x++){
        for(int y = min_y; y < max_y; y++){
            for(int z = min_z; z < max_z; z++){
                
                if(my_map[x][y][z].map_state == 0) continue;

                geometry_msgs::Point32 cur_pt;
                cur_pt.x = (x - offset_x) * map_resolusion;
                cur_pt.y = (y - offset_y) * map_resolusion;
                cur_pt.z = z * map_resolusion;

                if(my_map[x][y][z].map_state == 1){
                    occ_pts.points.emplace_back(cur_pt);
                }
                else{
                    occ_exp_pts.points.emplace_back(cur_pt);
                }
            }
        }
    }
}
