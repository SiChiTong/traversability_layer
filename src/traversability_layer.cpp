#include <pluginlib/class_list_macros.h>
#include <traversability_layer/traversability_layer.hpp>

PLUGINLIB_EXPORT_CLASS(traversability_layer::TraversabilityLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using costmap_2d::FREE_SPACE;

namespace traversability_layer{

TraversabilityLayer::TraversabilityLayer(){
    input_tm_ = grid_map_msgs::GridMap();
    new_tm_ = false;
    index_of_interest = 0;
}

void TraversabilityLayer::onInitialize(){
    ros::NodeHandle nh("~/" + name_);

    tm_sub_ = nh.subscribe("/traversability_estimation/traversability_map", 1, &TraversabilityLayer::traversabilityMapCallback, this);
    ROS_WARN("TraversabilityLayer is a cool layer that transforms traversability_maps to costmaps!");
}


void TraversabilityLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y){
    /*
    double mark_x = robot_x + cos(robot_yaw), mark_y = robot_y + sin(robot_yaw);
    unsigned int mx;
    unsigned int my;
    if(worldToMap(mark_x, mark_y, mx, my)){
        setCost(mx, my, LETHAL_OBSTACLE);
    }

    *min_x = std::min(*min_x, mark_x);
    *min_y = std::min(*min_y, mark_y);
    *max_x = std::max(*max_x, mark_x);
    *max_y = std::max(*max_y, mark_y);
    */
    //setCost(0, 0, LETHAL_OBSTACLE);
    if(new_tm_){
        bool new_search_required = false;
        if(index_of_interest < input_tm_.layers.size()){
            if(input_tm_.layers[index_of_interest].compare("traversability") != 0){
                new_search_required = true;
            }
        }
        else{
            new_search_required = true;
        }
        if(new_search_required){
        index_of_interest = 0;
            for(unsigned i = 0; i < input_tm_.layers.size(); i++){
                if(input_tm_.layers[i].compare("traversability") == 0){
                    index_of_interest = i;
                    ROS_ERROR("%u",index_of_interest);
                    ROS_ERROR("%s",input_tm_.layers[index_of_interest].c_str());
                    break;
                }
            }
        }
        //TODO use the index_of_interest and then switch the flag to false
        new_tm_ = false;
    }
}

void TraversabilityLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j){

    for (int j = min_j; j < max_j; j++){
        for (int i = min_i; i < max_i; i++){
            int index = getIndex(i, j);
            if (costmap_[index] != NO_INFORMATION){
                master_grid.setCost(i, j, costmap_[index]); 
            }
        }
    }
}

void TraversabilityLayer::matchSize(){
    Costmap2D* master = layered_costmap_->getCostmap();
    resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
        master->getOriginX(), master->getOriginY());
}

void TraversabilityLayer::traversabilityMapCallback(const grid_map_msgs::GridMap& map){
    input_tm_ = grid_map_msgs::GridMap(map);
    ROS_INFO("Updated traversability map!");
    new_tm_ = true;
}

}
