#ifndef TRAVERSABILITY_LAYER_H_
#define TRAVERSABILITY_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <grid_map_msgs/GridMap.h>
#include <costmap_2d/layered_costmap.h>
#include <dynamic_reconfigure/server.h>
#include <costmap_2d/GenericPluginConfig.h>

namespace traversability_layer{

    class TraversabilityLayer : public costmap_2d::Layer, public costmap_2d::Costmap2D{

        public:
            TraversabilityLayer();

            virtual void onInitialize();
            virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y);
            virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
            bool isDiscretized(){ return true; }
            virtual void matchSize();

        protected:
            void traversabilityMapCallback(const grid_map_msgs::GridMap& map);

            bool new_tm_;
            ros::Subscriber tm_sub_;
            unsigned index_of_interest;
            grid_map_msgs::GridMap input_tm_;
    };
}
#endif