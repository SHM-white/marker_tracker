//
// Created by mijiao on 23-12-10.
//

#include "enemy_params.h"

void EnemyParams::init(rclcpp::Node::SharedPtr _node) {
    node = _node;

    node->declare_parameter("/enemy/is3big", false);
    node->declare_parameter("/enemy/is4big", false);
    node->declare_parameter("/enemy/is5big", false);
}

bool EnemyParams::getIsBig(int id) {
    return node->get_parameter("/enemy/is" + std::to_string(id) + "big").as_bool();
}
