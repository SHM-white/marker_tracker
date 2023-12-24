//
// Created by mijiao on 23-12-24.
//

#ifndef MARKER_TRACKER_ENEMY_PARAMS_H
#define MARKER_TRACKER_ENEMY_PARAMS_H

#include <string>

#include <boost/serialization/singleton.hpp>

#include <rclcpp/rclcpp.hpp>

class EnemyParams : public boost::serialization::singleton<EnemyParams> {
private:
    rclcpp::Node::SharedPtr node;
public:
    void init(rclcpp::Node::SharedPtr _node);

    bool getIsBig(int id);
};

#define enemyParams EnemyParams::get_mutable_instance()

#endif //MARKER_TRACKER_ENEMY_PARAMS_H
