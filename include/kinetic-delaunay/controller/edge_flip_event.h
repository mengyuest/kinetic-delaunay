#ifndef KINETIC_DELAUNAY_EDGE_FLIP_EVENT_H
#define KINETIC_DELAUNAY_EDGE_FLIP_EVENT_H

#include "kinetic-delaunay/controller/my_event.h"

class EdgeFlipEvent : public MyEvent {
private:
    std::weak_ptr<Triangulation> triangulation_;
    std::shared_ptr<Triangulation::Edge> edge_;
public:
    EdgeFlipEvent(std::shared_ptr<Triangulation::Edge> &edge, std::weak_ptr<Triangulation> triangulation, double time);

    void Process() override;
};

#endif //KINETIC_DELAUNAY_EDGE_FLIP_EVENT_H
