#ifndef KINETIC_DELAUNAY_MANAGER_H
#define KINETIC_DELAUNAY_MANAGER_H

#include <opencv2/opencv.hpp>
#include "kinetic-delaunay/data/my_point.h"
#include "kinetic-delaunay/data/triangulation.h"
#include "kinetic-delaunay/controller/reflect_event.h"
#include "kinetic-delaunay/controller/collide_event.h"
#include "kinetic-delaunay/controller/edge_flip_event.h"
#include "kinetic-delaunay/data/poly.h"
#include <fstream>
#include <map>

using namespace cv;

struct Cmp {
    bool operator()(const MyEvent *lhs, const MyEvent *rhs) const {
        if (lhs->GetTime() != rhs->GetTime()) {
            return lhs->GetTime() < rhs->GetTime();
        } else if (lhs->vertices_.size() != rhs->vertices_.size()) {
            return lhs->vertices_.size() > rhs->vertices_.size();
        } else {
            return lhs->id_ < rhs->id_;
        }
    }
};

class Manager {
public:
    Manager(double bound, double delta, double factor);
    void Load(String path);
    void Update();
    const Mat Render();

private:
    int height_;
    int width_;
    double delta_;
    double time_;
    double bound_;
    double factor_;

    std::vector<std::shared_ptr<MyPoint>> my_points;
    std::shared_ptr<Triangulation> triangulation;
    std::set<MyEvent *, Cmp> event_queue;
    std::map<int, std::shared_ptr<ReflectEvent>> horizontal_reflect_events;
    std::map<int, std::shared_ptr<ReflectEvent>> vertical_reflect_events;
    std::map<std::pair<int, int>, std::shared_ptr<CollideEvent>> collide_events;
    std::map<int, std::shared_ptr<EdgeFlipEvent>> edge_flip_events;

    void InitQueue();
    void UpdatePoints(double dt);
    void NewReflectInstance(shared_ptr<Triangulation::Vertex> &p, bool is_horizontal);
    void RemoveReflectEvents(shared_ptr<Triangulation::Vertex> &p);
    void NewCollideInstance(shared_ptr<Triangulation::Vertex> &p1, shared_ptr<Triangulation::Vertex> &p2);
    bool RemoveCollideEvents(shared_ptr<Triangulation::Vertex> &p1, shared_ptr<Triangulation::Vertex> &p2);
    void NewEdgeFlipInstance(shared_ptr<Triangulation::Edge> &edge);
    void RemoveEdgeFlipEvents(weak_ptr<Triangulation::Edge> &edge);
    void Invalidate(shared_ptr<Triangulation::Vertex> &p);
    void InvalidateEdge(shared_ptr<Triangulation::Edge> &edge);
    std::map<int, shared_ptr<Triangulation::Vertex>> GetConvexHullVertices();
};

#endif //KINETIC_DELAUNAY_MANAGER_H
