#include <pluginlib/class_list_macros.h>
#include <traversability_layer/traversability_layer.hpp>

PLUGINLIB_EXPORT_CLASS(traversability_layer::TraversabilityLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using costmap_2d::FREE_SPACE;

namespace traversability_layer{

TraversabilityLayer::TraversabilityLayer(){
    input_tm_ = grid_map_msgs::GridMap();
    current_ = true;
    new_tm_ = false;
    index_of_interest = 0;
}

void TraversabilityLayer::onInitialize(){
    ros::NodeHandle nh("~/" + name_);
    matchSize();

    tm_sub_ = nh.subscribe("/traversability_estimation/traversability_map", 1, &TraversabilityLayer::traversabilityMapCallback, this);
    ROS_WARN("TraversabilityLayer is a cool layer that transforms traversability_maps to costmaps!");
}


void TraversabilityLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y){
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
                    break;
                }
            }
        }

        unsigned int mx;
        unsigned int my;
        float resolution = input_tm_.info.resolution; // meters / cell
        float length_x = input_tm_.info.length_x; // meters
        float length_y = input_tm_.info.length_y; // meters
        unsigned data_offset = input_tm_.data[index_of_interest].layout.data_offset; // zero for traversability
        unsigned index_x = input_tm_.data[index_of_interest].layout.dim[1].size; // assume row_index TODO add a check here
        unsigned index_y = input_tm_.data[index_of_interest].layout.dim[0].size; // assume column_index TODO add a check here
        for(unsigned i = 0; i < input_tm_.data[index_of_interest].data.size(); i++){
            float cellv = input_tm_.data[index_of_interest].data[i];
            // If not NaN
            if(cellv == cellv){
                double cx = resolution * int(index_x / 2 - i % index_x);
                double cy = resolution * int(index_y / 2 - i / index_y);
                double d = sqrt(pow(cx - robot_x, 2) + pow(cy - robot_y, 2));
                double cellx = d * cos(atan(cy/cx)) + getSizeInMetersX()/2;
                double celly = d * sin(atan(cy/cx)) + getSizeInMetersY()/2;
                if(worldToMap(cellx, celly, mx, my)){
                    // Works with the latest (slightly customized!) traversability expression:
                    // 1 - (1.0 / 3.0) * (traversability_slope + traversability_step + traversability_roughness)
                    double cost = std::min(double(LETHAL_OBSTACLE), 2.0 * LETHAL_OBSTACLE * (cellv - 0.1));
                    if (cost <= LETHAL_OBSTACLE / 2.0) {
                        cost = 0.0;
                    }
                    setCost(mx, my, cost);
                    // UNOPTIMIZED!
                    *min_x = std::min(*min_x, -100.0);
                    *min_y = std::min(*min_y, -100.0);
                    *max_x = std::max(*max_x, 100.0);
                    *max_y = std::max(*max_y, 100.0);
                    /*
                    *min_x = std::min(*min_x, double(mx));
                    *min_y = std::min(*min_y, double(my));
                    *max_x = std::max(*max_x, double(mx));
                    *max_y = std::max(*max_y, double(my));
                    */
                    
                }
            }
        }
        new_tm_ = false;
    }
}

void TraversabilityLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j){

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
    new_tm_ = true;
    resetMaps();
}

}
