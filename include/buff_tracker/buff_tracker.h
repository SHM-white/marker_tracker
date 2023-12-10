//
// Created by mijiao on 23-12-8.
//

#ifndef MARKER_TRACKER_BUFF_TRACKER_H
#define MARKER_TRACKER_BUFF_TRACKER_H

#include "tracker/tracker.h"

class BuffTracker : public Tracker{
private:

public:
    TrackerResult track(geometry_msgs::msg::Pose pose) override;
};


#endif //MARKER_TRACKER_BUFF_TRACKER_H
