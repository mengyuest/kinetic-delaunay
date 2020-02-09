#ifndef KINETIC_DELAUNAY_MY_EVENT_H
#define KINETIC_DELAUNAY_MY_EVENT_H

#include <memory>
#include <set>
#include "kinetic-delaunay/data/my_point.h"
#include "kinetic-delaunay/data/triangulation.h"

class MyEvent {
private:
    double time_;
    static int num_events_;
public:
    int id_;
    std::vector<std::shared_ptr<Triangulation::Vertex>> vertices_;
    std::vector<std::shared_ptr<Triangulation::Edge>> edges_;

    MyEvent(double time);
    const double GetTime() const;
    virtual void Process() {};
};

#endif //KINETIC_DELAUNAY_MY_EVENT_H
