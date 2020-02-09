#include "kinetic-delaunay/controller/my_event.h"

int MyEvent::num_events_ = 0;

MyEvent::MyEvent(double time) : time_(time), id_(++num_events_) {
    vertices_.clear();
    edges_.clear();
    num_events_ = num_events_ % 2147483647;
}

const double MyEvent::GetTime() const{
    return time_;
}
