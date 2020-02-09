#include "kinetic-delaunay/controller/edge_flip_event.h"
#include <iostream>

EdgeFlipEvent::EdgeFlipEvent(std::shared_ptr<Triangulation::Edge> &edge,
                             std::weak_ptr<Triangulation> tri, double time) :
        MyEvent(time), triangulation_(tri), edge_(edge) {
    edges_.push_back(edge);
}

void EdgeFlipEvent::Process() {
    triangulation_.lock()->EdgeFlip(edge_);
}