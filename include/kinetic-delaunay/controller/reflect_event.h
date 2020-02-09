#ifndef KINETIC_DELAUNAY_REFLECT_EVENT_H
#define KINETIC_DELAUNAY_REFLECT_EVENT_H

#include "kinetic-delaunay/data/my_point.h"
#include "kinetic-delaunay/controller/my_event.h"


class ReflectEvent : public MyEvent {
private:
    shared_ptr<Triangulation::Vertex> vertex_;
    bool is_horizontal_;
public:
    ReflectEvent(shared_ptr<Triangulation::Vertex> &vertex, bool is_horizontal, double time);
    void Process() override;
};

#endif //KINETIC_DELAUNAY_REFLECT_EVENT_H
