#ifndef KINETIC_DELAUNAY_TRIANGULATION_H
#define KINETIC_DELAUNAY_TRIANGULATION_H

#include <memory>
#include <set>
#include <queue>
#include <vector>
#include "kinetic-delaunay/data/my_point.h"

using namespace std;

class Triangulation {
public:
    class Edge;

    class LocationTriangle;

    class Face;

    class Vertex {
    public:
        shared_ptr<MyPoint> point_;
        shared_ptr<Edge> edge_;

        Vertex(shared_ptr<MyPoint> &point);
    };

    class Edge {
    private:
        static int num_edges_;
    public:
        int id_;
        weak_ptr<Vertex> vertex_;
        weak_ptr<Edge> pair_;
        weak_ptr<Edge> next_;
        shared_ptr<Face> face_;

        Edge();
        bool operator<(const Edge &right) const;
    };

    class Face {
    public:
        weak_ptr<Edge> edge_;
        shared_ptr<LocationTriangle> location_triangle_;
    };

    class LocationTriangle {
    public:
        weak_ptr<MyPoint> a_, b_, c_;
        weak_ptr<Face> face_;
        vector<weak_ptr<LocationTriangle>> children_;

        LocationTriangle(shared_ptr<Face> &face);

        void AddChild(weak_ptr<LocationTriangle> &child);

        weak_ptr<Face> GetFace(shared_ptr<MyPoint> &point);

    };

    set<shared_ptr<Face>> faces_;
    set<shared_ptr<Vertex>> vertices_;
    set<shared_ptr<Vertex>> inner_vertices_;
    set<shared_ptr<Vertex>> boundary_vertices_;
    set<shared_ptr<Edge>> edges_;
    set<shared_ptr<LocationTriangle>> location_triangles_;
    bool maintain_locations_ = true;

    Triangulation(vector<shared_ptr<MyPoint>> my_points, double bound, double factor);
    set<shared_ptr<Edge>> GetOutgoingEdges(shared_ptr<Vertex> &vertex);
    void EdgeFlip(shared_ptr<Edge> &edge);

private:
    shared_ptr<Vertex> NewVertex(shared_ptr<MyPoint> &p,  bool is_inner);
    shared_ptr<Edge> NewEdge();
    shared_ptr<Face> NewFace();
    shared_ptr<LocationTriangle> NewLocationTriangle(shared_ptr<Face> &f);
};

#endif //KINETIC_DELAUNAY_TRIANGULATION_H
