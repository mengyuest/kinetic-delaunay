#include "kinetic-delaunay/controller/collide_event.h"
#include <math.h>

CollideEvent::CollideEvent(shared_ptr<Triangulation::Vertex> &vertex_0, shared_ptr<Triangulation::Vertex> &vertex_1, double time) :
        MyEvent(time), vertex_0_(vertex_0), vertex_1_(vertex_1) {
    vertices_.push_back(vertex_0);
    vertices_.push_back(vertex_1);
}

void CollideEvent::Process() {
    double theta = atan2(vertex_1_->point_->y_ - vertex_0_->point_->y_, vertex_1_->point_->x_ - vertex_0_->point_->x_);
    double pi_plus_theta = M_PI + theta;
    double cos_0 = cos(theta);
    double sin_0 = sin(theta);
    double cos_1 = cos(pi_plus_theta);
    double sin_1 = sin(pi_plus_theta);
    double v_x_0 = vertex_0_->point_->v_x_ * cos_0 * cos_0 + vertex_0_->point_->v_y_ * sin_0 * cos_0;
    double v_y_0 = vertex_0_->point_->v_x_ * cos_0 * sin_0 + vertex_0_->point_->v_y_ * sin_0 * sin_0;
    double v_x_1 = vertex_1_->point_->v_x_ * cos_1 * cos_1 + vertex_1_->point_->v_y_ * sin_1 * cos_1;
    double v_y_1 = vertex_1_->point_->v_x_ * cos_1 * sin_1 + vertex_1_->point_->v_y_ * sin_1 * sin_1;
    vertex_0_->point_->v_x_ += v_x_1 - v_x_0;
    vertex_0_->point_->v_y_ += v_y_1 - v_y_0;
    vertex_1_->point_->v_x_ += v_x_0 - v_x_1;
    vertex_1_->point_->v_y_ += v_y_0 - v_y_1;

    if (std::fabs(vertex_0_->point_->v_x_) < MyPoint::kInfinitesimal) {
        vertex_0_->point_->v_x_ = 0;
    }
    if (std::fabs(vertex_0_->point_->v_y_) < MyPoint::kInfinitesimal) {
        vertex_0_->point_->v_y_ = 0;
    }
    if (std::fabs(vertex_1_->point_->v_x_) < MyPoint::kInfinitesimal) {
        vertex_1_->point_->v_x_ = 0;
    }
    if (std::fabs(vertex_1_->point_->v_y_) < MyPoint::kInfinitesimal) {
        vertex_1_->point_->v_y_ = 0;
    }
}
