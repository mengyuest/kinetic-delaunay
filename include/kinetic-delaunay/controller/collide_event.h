#ifndef KINETIC_DELAUNAY_COLLIDE_EVENT_H
#define KINETIC_DELAUNAY_COLLIDE_EVENT_H

#include "kinetic-delaunay/controller/my_event.h"
#include "kinetic-delaunay/data/my_point.h"

class CollideEvent : public MyEvent {
private:
    shared_ptr<Triangulation::Vertex> vertex_0_;
    shared_ptr<Triangulation::Vertex> vertex_1_;
public:
    CollideEvent(shared_ptr<Triangulation::Vertex> &vertex_0, shared_ptr<Triangulation::Vertex> &vertex_1, double time);

    void Process() override;
};

#endif //KINETIC_DELAUNAY_COLLIDE_EVENT_H
