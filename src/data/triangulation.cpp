#include "kinetic-delaunay/data/triangulation.h"

Triangulation::Vertex::Vertex(shared_ptr<MyPoint> &point) : point_(point) {}

int Triangulation::Edge::num_edges_ = 0;

Triangulation::Edge::Edge() : id_(++num_edges_) {}

bool Triangulation::Edge::operator<(const Triangulation::Edge &right) const {
    return this->id_ < right.id_;
}


Triangulation::LocationTriangle::LocationTriangle(std::shared_ptr<Triangulation::Face> &face) {
    this->face_ = face;
    this->a_ = face->edge_.lock()->vertex_.lock()->point_;
    this->b_ = face->edge_.lock()->next_.lock()->vertex_.lock()->point_;
    this->c_ = face->edge_.lock()->next_.lock()->next_.lock()->vertex_.lock()->point_;
}

void Triangulation::LocationTriangle::AddChild(std::weak_ptr<LocationTriangle> &child) {
    this->children_.push_back(child);
}

std::weak_ptr<Triangulation::Face> Triangulation::LocationTriangle::GetFace(std::shared_ptr<MyPoint> &point) {
    for (std::weak_ptr<LocationTriangle> child : children_) {
        if (point->InTriangle(*(child.lock()->a_.lock()),
                              *(child.lock()->b_.lock()),
                              *(child.lock()->c_.lock()))) {
            return child.lock()->GetFace(point);
        }
    }
    return this->face_;
}

shared_ptr<Triangulation::Vertex> Triangulation::NewVertex(shared_ptr<MyPoint> &p, bool is_inner) {
    shared_ptr<Vertex> vertex = make_shared<Triangulation::Vertex>(p);
    this->vertices_.insert(vertex);
    if (is_inner){
        this->inner_vertices_.insert(vertex);
    }
    return vertex;
}

shared_ptr<Triangulation::Edge> Triangulation::NewEdge() {
    auto edge = make_shared<Edge>();
    this->edges_.insert(edge);
    return edge;
}

shared_ptr<Triangulation::Face> Triangulation::NewFace() {
    auto face = make_shared<Face>();
    this->faces_.insert(face);
    return face;
}

shared_ptr<Triangulation::LocationTriangle>
Triangulation::NewLocationTriangle(std::shared_ptr<Triangulation::Face> &f) {
    auto location_triangle = make_shared<LocationTriangle>(f);
    this->location_triangles_.insert(location_triangle);
    return location_triangle;
}

Triangulation::Triangulation(std::vector<shared_ptr<MyPoint>> my_points, double bound, double factor) {
    shared_ptr<Face> f = this->NewFace();

    auto p0 = make_shared<MyPoint>(-factor * bound, -factor * bound, 0, 0);
    auto p1 = make_shared<MyPoint>(factor * bound, -factor * bound, 0, 0);
    auto p2 = make_shared<MyPoint>(0, factor * bound, 0, 0);
    shared_ptr<Vertex> v0 = NewVertex(p0, false);
    shared_ptr<Vertex> v1 = NewVertex(p1, false);
    shared_ptr<Vertex> v2 = NewVertex(p2, false);

    boundary_vertices_.insert(v0);
    boundary_vertices_.insert(v1);
    boundary_vertices_.insert(v2);

    shared_ptr<Edge> e0 = NewEdge();
    e0->face_ = f;
    e0->vertex_ = v0;

    shared_ptr<Edge> e1 = NewEdge();
    e1->face_ = f;
    e1->vertex_ = v1;

    shared_ptr<Edge> e2 = NewEdge();
    e2->face_ = f;
    e2->vertex_ = v2;

    f->edge_ = e0;

    v0->edge_ = e0;
    v1->edge_ = e1;
    v2->edge_ = e2;

    e0->next_ = e1;
    e1->next_ = e2;
    e2->next_ = e0;

    auto root = NewLocationTriangle(f);
    f->location_triangle_ = root;

    for (shared_ptr<MyPoint> p : my_points) {
        f = root->GetFace(p).lock();
        shared_ptr<Vertex> v = NewVertex(p, true);
        queue<shared_ptr<Edge>> suspect;

        vector<shared_ptr<Edge>> es;
        for (int i = 0; i < 6; i++) {
            es.push_back(NewEdge());
        }
        v->edge_ = es[1];
        shared_ptr<Edge> e = f->edge_.lock();

        suspect.push(e);
        shared_ptr<Edge> ne = e->next_.lock();
        e->next_ = es[0];
        e->next_.lock()->face_ = e->face_;
        e->next_.lock()->vertex_ = ne->vertex_;
        e->next_.lock()->pair_ = es[3];
        e->next_.lock()->next_ = es[1];
        e->next_.lock()->next_.lock()->face_ = e->face_;
        e->next_.lock()->next_.lock()->vertex_ = v;
        e->next_.lock()->next_.lock()->pair_ = es[4];
        e->next_.lock()->next_.lock()->next_ = e;
        e = ne;

        suspect.push(e);
        ne = e->next_.lock();
        shared_ptr<Face> f1 = NewFace();
        f1->edge_ = e;
        e->face_ = f1;
        e->next_ = es[2];
        e->next_.lock()->face_ = e->face_;
        e->next_.lock()->vertex_ = ne->vertex_;
        e->next_.lock()->pair_ = es[5];
        e->next_.lock()->next_ = es[3];
        e->next_.lock()->next_.lock()->face_ = e->face_;
        e->next_.lock()->next_.lock()->vertex_ = v;
        e->next_.lock()->next_.lock()->pair_ = es[0];
        e->next_.lock()->next_.lock()->next_ = e;
        e = ne;

        suspect.push(e);
        ne = e->next_.lock();
        shared_ptr<Face> f2 = NewFace();
        f2->edge_ = e;
        e->face_ = f2;
        e->next_ = es[4];
        e->next_.lock()->face_ = e->face_;
        e->next_.lock()->vertex_ = ne->vertex_;
        e->next_.lock()->pair_ = es[1];
        e->next_.lock()->next_ = es[5];
        e->next_.lock()->next_.lock()->face_ = e->face_;
        e->next_.lock()->next_.lock()->vertex_ = v;
        e->next_.lock()->next_.lock()->pair_ = es[2];
        e->next_.lock()->next_.lock()->next_ = e;

        auto t0 = NewLocationTriangle(f);
        auto t1 = NewLocationTriangle(f1);
        auto t2 = NewLocationTriangle(f2);

        auto wt0 = weak_ptr<LocationTriangle>(t0);
        auto wt1 = weak_ptr<LocationTriangle>(t1);
        auto wt2 = weak_ptr<LocationTriangle>(t2);

        f->location_triangle_->AddChild(wt0);
        f->location_triangle_->AddChild(wt1);
        f->location_triangle_->AddChild(wt2);

        f->location_triangle_ = t0;
        f1->location_triangle_ = t1;
        f2->location_triangle_ = t2;

        while (suspect.empty() == false) {
            e = suspect.front();
            suspect.pop();
            if (e->pair_.expired()) continue;
            if (p->InCircle(*(e->pair_.lock()->vertex_.lock()->point_),
                            *(e->pair_.lock()->next_.lock()->vertex_.lock()->point_),
                            *(e->pair_.lock()->next_.lock()->next_.lock()->vertex_.lock()->point_)) > 0) {
                EdgeFlip(e);
                suspect.push(e->next_.lock());
                suspect.push(e->pair_.lock()->next_.lock()->next_.lock());
            }
        }
    }

    maintain_locations_ = false;
}

void Triangulation::EdgeFlip(shared_ptr<Edge> &edge) {
    shared_ptr<Edge> f = edge->pair_.lock();
    edge->face_->edge_ = edge;
    f->face_->edge_ = f;

    shared_ptr<Edge> a = edge->next_.lock();
    shared_ptr<Edge> b = a->next_.lock();
    shared_ptr<Edge> c = edge->pair_.lock()->next_.lock();
    shared_ptr<Edge> d = c->next_.lock();

    edge->vertex_ = b->vertex_;
    edge->next_ = d;
    f->vertex_ = d->vertex_;
    f->next_ = b;

    a->next_ = edge;
    a->vertex_.lock()->edge_ = a;
    c->next_ = f;
    c->vertex_.lock()->edge_ = c;
    b->next_ = c;
    b->face_ = c->face_;
    d->next_ = a;
    d->face_ = a->face_;

    if (maintain_locations_) {
        auto et = NewLocationTriangle(edge->face_);
        auto ft = NewLocationTriangle(f->face_);
        auto wet = weak_ptr<LocationTriangle>(et);
        auto wft = weak_ptr<LocationTriangle>(ft);
        edge->face_->location_triangle_->AddChild(wet);
        edge->face_->location_triangle_->AddChild(wft);
        f->face_->location_triangle_->AddChild(wet);
        f->face_->location_triangle_->AddChild(wft);
        edge->face_->location_triangle_ = et;
        f->face_->location_triangle_ = ft;
    }
}

set<shared_ptr<Triangulation::Edge>> Triangulation::GetOutgoingEdges(shared_ptr<Triangulation::Vertex> &vertex) {
    set<shared_ptr<Edge>> answer;
    auto first = vertex->edge_;
    auto cursor = first;
    answer.insert(cursor);
    while (cursor->pair_.lock()->next_.lock()->id_ != first->id_) {
        cursor = cursor->pair_.lock()->next_.lock();
        answer.insert(cursor);
    }
    return answer;
}