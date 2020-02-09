#include "kinetic-delaunay/controller/reflect_event.h"

ReflectEvent::ReflectEvent(shared_ptr<Triangulation::Vertex> &vertex, bool is_horizontal, double time) :
        MyEvent(time), vertex_(vertex), is_horizontal_(is_horizontal) {
    vertices_.push_back(vertex);
}

void ReflectEvent::Process() {
    if (is_horizontal_) {
        vertex_->point_->v_x_ *= -1;
    } else {
        vertex_->point_->v_y_ *= -1;
    }
}