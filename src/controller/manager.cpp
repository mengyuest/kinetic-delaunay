#include "kinetic-delaunay/controller/manager.h"
#include <math.h>

using namespace cv;
using namespace std;

Manager::Manager(double bound, double delta, double factor) :
        height_(bound * 2), width_(bound * 2), bound_(bound), delta_(delta), factor_(factor), time_(0) {}

void Manager::Load(String path) {
    ifstream fin(path);
    int num_points, radius;
    int x, y, vx, vy;
    fin >> num_points >> radius;
    for (int i = 0; i < num_points; i++) {
        fin >> x >> y >> vx >> vy;
        my_points.push_back(std::make_shared<MyPoint>(x, y, vx, vy, radius));
    }
    fin.close();

    triangulation = make_shared<Triangulation>(my_points, bound_, factor_);
    InitQueue();
}

void Manager::InitQueue() {
    for (auto v1 : triangulation->inner_vertices_) {
        NewReflectInstance(v1, true);
        NewReflectInstance(v1, false);
        for (auto v2: triangulation->inner_vertices_) {
            if (v1->point_->id_ >= v2->point_->id_) { //TODO(yue) only handle for half (always p1.id < p2.id)
                continue;
            }

            NewCollideInstance(v1, v2);
        }
    }
    for (auto edge : triangulation->edges_) {
        if (edge->pair_.expired()) {
            continue;
        }
        if (edge_flip_events.find(edge->pair_.lock()->id_) != edge_flip_events.end()) {
            continue;
        }
        NewEdgeFlipInstance(edge);
    }
}

bool CmpByProduct(const shared_ptr<Triangulation::Vertex> &a, const shared_ptr<Triangulation::Vertex> &b) {
    double d_ax = a->point_->x_ - MyPoint::base_x_;
    double d_ay = a->point_->y_ - MyPoint::base_y_;
    double d_bx = b->point_->x_ - MyPoint::base_x_;
    double d_by = b->point_->y_ - MyPoint::base_y_;
    double dot_product_a = d_ax / (sqrt(d_ax * d_ax + d_ay * d_ay) + MyPoint::kInfinitesimal);
    double dot_product_b = d_bx / (sqrt(d_bx * d_bx + d_by * d_by) + MyPoint::kInfinitesimal);

    if (dot_product_a != dot_product_b) {
        return dot_product_a > dot_product_b;
    } else {
        return -d_ax * d_by + d_ay * d_bx > 0;
    }
}

std::map<int, shared_ptr<Triangulation::Vertex>> Manager::GetConvexHullVertices() {

    std::vector<shared_ptr<Triangulation::Vertex>> convex_hull_vertices;
    std::vector<shared_ptr<Triangulation::Vertex>> sorting_vertices;

    shared_ptr<Triangulation::Vertex> vertex_with_lowest_y;
    for (auto vertex: triangulation->inner_vertices_) {
        if (vertex_with_lowest_y == nullptr || vertex->point_->y_ < vertex_with_lowest_y->point_->y_ ||
            (vertex->point_->y_ == vertex_with_lowest_y->point_->y_ && vertex->point_->x_ < vertex_with_lowest_y->point_->x_)) {
            vertex_with_lowest_y = vertex;
        }
        sorting_vertices.push_back(vertex);
    }
    MyPoint::base_x_ = vertex_with_lowest_y->point_->x_;
    MyPoint::base_y_ = vertex_with_lowest_y->point_->y_;

    for (auto it = sorting_vertices.begin(); it != sorting_vertices.end(); it++) {
        if ((*it)->point_->id_ == vertex_with_lowest_y->point_->id_) {
            sorting_vertices.erase(it);
            break;
        }
    }

    sort(sorting_vertices.begin(), sorting_vertices.end(), CmpByProduct);

    convex_hull_vertices.push_back(vertex_with_lowest_y);
    for (auto vertex: sorting_vertices) {
        convex_hull_vertices.push_back(vertex);
        if (convex_hull_vertices.size() > 2) {
            auto last = convex_hull_vertices[convex_hull_vertices.size() - 2];
            auto second_last = convex_hull_vertices[convex_hull_vertices.size() - 3];

            double direction = (vertex->point_->x_ - last->point_->x_) * (vertex->point_->y_ - second_last->point_->y_) -
                               (vertex->point_->y_ - last->point_->y_) * (vertex->point_->x_ - second_last->point_->x_);
            while (direction > 0) {
                convex_hull_vertices.erase(convex_hull_vertices.end() - 2);
                if (convex_hull_vertices.size() == 2) {
                    break;
                }
                last = convex_hull_vertices[convex_hull_vertices.size() - 2];
                second_last = convex_hull_vertices[convex_hull_vertices.size() - 3];
                direction = (vertex->point_->x_ - last->point_->x_) * (vertex->point_->y_ - second_last->point_->y_) -
                            (vertex->point_->y_ - last->point_->y_) * (vertex->point_->x_ - second_last->point_->x_);
            }
        }
    }
    std::map<int, shared_ptr<Triangulation::Vertex>> convex_set;
    for (auto vertex: convex_hull_vertices) {
        convex_set.insert(std::make_pair(vertex->point_->id_, vertex));
    }

    return convex_set;
}

void Manager::Update() {
    double final_time = time_ + delta_;
    //TODO(yue) event queue dispatcher
    while (event_queue.empty() == false) {
        MyEvent *current_event = *(event_queue.begin());
        if (current_event->GetTime() > final_time) {
            break;
        }
        UpdatePoints(current_event->GetTime() - time_);
        current_event->Process();

        std::vector<shared_ptr<Triangulation::Vertex>> tmp_vertices;
        std::vector<shared_ptr<Triangulation::Edge>> tmp_edges;

        for (auto vertex:current_event->vertices_) {
            tmp_vertices.push_back(vertex);
        }

        for (auto edge:current_event->edges_) {
            tmp_edges.push_back(edge);
        }

        for (auto vertex:tmp_vertices) {
            Invalidate(vertex);
        }

        for (auto edge: tmp_edges) {
            InvalidateEdge(edge);
        }
    }

    //TODO(yue) convex hull
    std::set<shared_ptr<Triangulation::Vertex>> vertices_to_delete;
    auto convex_set = GetConvexHullVertices();
    for (auto it: horizontal_reflect_events) {
        if (convex_set.find(it.first) == convex_set.end()) {
            vertices_to_delete.insert(it.second->vertices_[0]);
        }
    }
    for (auto it: vertical_reflect_events) {
        if (convex_set.find(it.first) == convex_set.end()) {
            vertices_to_delete.insert(it.second->vertices_[0]);
        }
    }
    for (auto vertex: vertices_to_delete) {
        RemoveReflectEvents(vertex);
    }
    for (auto kv:convex_set) {
        if (horizontal_reflect_events.find(kv.first) == horizontal_reflect_events.end()) {
            NewReflectInstance(kv.second, true);
            NewReflectInstance(kv.second, false);
        }
    }

    //TODO(yue) vertices pairs forming in Delaunay edges
    std::map<int, std::map<int,
            std::pair<std::shared_ptr<Triangulation::Vertex>,
                    std::shared_ptr<Triangulation::Vertex>>>> delaunay_vertices_pairs;
    for (auto edge: triangulation->edges_) {
        if (edge->pair_.expired()) continue;

        auto v0 = edge->vertex_.lock();
        auto v1 = edge->pair_.lock()->vertex_.lock();
        if (triangulation->boundary_vertices_.find(v0) != triangulation->boundary_vertices_.end()
            || triangulation->boundary_vertices_.find(v1) != triangulation->boundary_vertices_.end()) {
            continue;
        }

        delaunay_vertices_pairs[v0->point_->id_][v1->point_->id_] = std::make_pair(v0, v1);
        delaunay_vertices_pairs[v1->point_->id_][v0->point_->id_] = std::make_pair(v1, v0);
    }

    //TODO(yue) delete old vertices pairs
    std::vector<std::pair<shared_ptr<Triangulation::Vertex>, shared_ptr<Triangulation::Vertex>>> delPointPairs;
    for (auto it: collide_events) {
        if (delaunay_vertices_pairs.find(it.second->vertices_[0]->point_->id_) == delaunay_vertices_pairs.end() ||
            delaunay_vertices_pairs[it.second->vertices_[0]->point_->id_].find(it.second->vertices_[1]->point_->id_)
            == delaunay_vertices_pairs[it.second->vertices_[0]->point_->id_].end()) {
            delPointPairs.push_back(std::make_pair(
                    it.second->vertices_[0],
                    it.second->vertices_[1]
            ));
        }
    }

    for (auto pair: delPointPairs) {
        RemoveCollideEvents(pair.first, pair.second);
    }


    //TODO(yue) add new vertices pairs
    for (auto it0: delaunay_vertices_pairs) {
        for (auto v1: delaunay_vertices_pairs[it0.first]) {
            if (collide_events.find(std::make_pair(it0.first, v1.first))
                == collide_events.end()) {
                auto pair = delaunay_vertices_pairs[it0.first][v1.first];
                if (it0.first < v1.first) {
                    NewCollideInstance(pair.first, pair.second);
                }
            }
        }
    }

    UpdatePoints(final_time - time_);
}

void Manager::UpdatePoints(double dt) {
    for (shared_ptr<MyPoint> p: my_points) {
        p->x_ += p->v_x_ * dt;
        p->y_ += p->v_y_ * dt;
    }
    time_ += dt;
}

const Mat Manager::Render() {
    Mat img = Mat::zeros(height_, width_, CV_8UC3);
    //TODO(yue) draw edges
    for (shared_ptr<Triangulation::Edge> edge : triangulation->edges_) {
        auto p0 = edge->vertex_.lock()->point_;
        auto p1 = edge->next_.lock()->vertex_.lock()->point_;
        auto color = Scalar(0, 255, 0);
        int thickness = 2;

        if (edge->pair_.expired()) { //TODO(yue) boundary edges
            continue;
        }
        if (triangulation->boundary_vertices_.find(edge->vertex_.lock()) != triangulation->boundary_vertices_.end()
            || triangulation->boundary_vertices_.find(edge->next_.lock()->vertex_.lock()) !=
               triangulation->boundary_vertices_.end()
                ) {
            color = Scalar(255, 0, 0);
            thickness = 1;
        }

        line(img, Point(p0->x_ + bound_, -p0->y_ + bound_), Point(p1->x_ + bound_, -p1->y_ + bound_),
             color, thickness, LINE_AA);
    }

    //TODO(yue) draw points
    for (shared_ptr<Triangulation::Vertex> vertex : triangulation->inner_vertices_) {
        circle(img, Point(vertex->point_->x_ + bound_, -vertex->point_->y_ + bound_), vertex->point_->r_,
               Scalar(0, 0, 255), -1,
               LINE_AA);
//        putText(img, to_string(vertex->point_->id_),
//                Point(vertex->point_->x_ + bound - vertex->point_->r_, -vertex->point_->y_ + bound),
//                FONT_HERSHEY_DUPLEX, 0.4, Scalar(255, 255, 255));
    }
    return img;
}

void Manager::NewReflectInstance(shared_ptr<Triangulation::Vertex> &p, bool is_horizontal) {
    double estimated_time;
    if (is_horizontal) {
        if (p->point_->v_x_ > 0)
            estimated_time = time_ + (bound_ - p->point_->x_ - p->point_->r_) / p->point_->v_x_;
        else if (p->point_->v_x_ < 0)
            estimated_time = time_ - (p->point_->x_ - p->point_->r_ + bound_) / p->point_->v_x_;
        else
            return;

        auto reflect_event = make_shared<ReflectEvent>(
                p, is_horizontal, estimated_time);
        horizontal_reflect_events.insert(
                std::pair<int, std::shared_ptr<ReflectEvent>>(p->point_->id_, reflect_event));
        event_queue.insert(&*reflect_event);
    } else {
        if (p->point_->v_y_ > 0)
            estimated_time = time_ + (bound_ - p->point_->y_ - p->point_->r_) / p->point_->v_y_;
        else if (p->point_->v_y_ < 0)
            estimated_time = time_ - (bound_ + p->point_->y_ - p->point_->r_) / p->point_->v_y_;
        else
            return;
        auto reflect_event = make_shared<ReflectEvent>(
                p, is_horizontal, estimated_time);

        vertical_reflect_events.insert(
                std::pair<int, std::shared_ptr<ReflectEvent>>(p->point_->id_, reflect_event));
        event_queue.insert(&*reflect_event);
    }
}

void Manager::RemoveReflectEvents(shared_ptr<Triangulation::Vertex> &p) {
    auto it = horizontal_reflect_events.find(p->point_->id_);
    if (it != horizontal_reflect_events.end()) {
        event_queue.erase(&*(it->second));
        horizontal_reflect_events.erase(it);
    }
    it = vertical_reflect_events.find(p->point_->id_);
    if (it != vertical_reflect_events.end()) {
        event_queue.erase(&*(it->second));
        vertical_reflect_events.erase(it);
    }
}

void Manager::NewCollideInstance(shared_ptr<Triangulation::Vertex> &p1, shared_ptr<Triangulation::Vertex> &p2) {
    double d_x = p1->point_->x_ - p2->point_->x_;
    double d_y = p1->point_->y_ - p2->point_->y_;
    double d_v_x = p1->point_->v_x_ - p2->point_->v_x_;
    double d_v_y = p1->point_->v_y_ - p2->point_->v_y_;

    double a = d_v_x * d_v_x + d_v_y * d_v_y;
    if (a == 0)
        return;
    double b = d_x * d_v_x + d_y * d_v_y;
    double c = d_x * d_x + d_y * d_y - (p1->point_->r_ + p2->point_->r_) * (p1->point_->r_ + p2->point_->r_);

    double disc = b * b - a * c;
    if (disc <= 0)
        return;
    double estimated_time = (-b - sqrt(disc)) / a;
    if (estimated_time <= MyPoint::kInfinitesimal)
        return;
    auto collide_event = make_shared<CollideEvent>(
            p1, p2, time_ + estimated_time);

    collide_events.insert(
            std::pair<std::pair<int, int>,
                    std::shared_ptr<CollideEvent>>(
                    std::make_pair(p1->point_->id_, p2->point_->id_), collide_event));
    event_queue.insert(&*collide_event);

}

bool Manager::RemoveCollideEvents(shared_ptr<Triangulation::Vertex> &p1, shared_ptr<Triangulation::Vertex> &p2) {
    auto it = collide_events.find(std::make_pair(p1->point_->id_, p2->point_->id_));
    if (it != collide_events.end()) {
        event_queue.erase(&*(it->second));
        collide_events.erase(it);
        return true;
    }
    return false;
}

void Manager::NewEdgeFlipInstance(shared_ptr<Triangulation::Edge> &edge) {
    if (edge->pair_.expired()) {
        return;
    }

    auto a = edge->vertex_.lock()->point_;
    auto b = edge->next_.lock()->vertex_.lock()->point_;
    auto c = edge->next_.lock()->next_.lock()->vertex_.lock()->point_;
    auto d = edge->pair_.lock()->next_.lock()->next_.lock()->vertex_.lock()->point_;

    Poly ax(std::vector<double>{a->x_, a->v_x_});
    Poly bx(std::vector<double>{b->x_, b->v_x_});
    Poly cx(std::vector<double>{c->x_, c->v_x_});
    Poly dx(std::vector<double>{d->x_, d->v_x_});

    Poly ay(std::vector<double>{a->y_, a->v_y_});
    Poly by(std::vector<double>{b->y_, b->v_y_});
    Poly cy(std::vector<double>{c->y_, c->v_y_});
    Poly dy(std::vector<double>{d->y_, d->v_y_});

    Poly az = ax * ax + ay * ay;
    Poly bz = bx * bx + by * by;
    Poly cz = cx * cx + cy * cy;
    Poly dz = dx * dx + dy * dy;

    //TODO(yue) this can be smarter
    Poly in_circle = Det3(ax, ay, az, bx, by, bz, cx, cy, cz) -
                     Det3(ax, ay, az, bx, by, bz, dx, dy, dz) +
                     Det3(ax, ay, az, cx, cy, cz, dx, dy, dz) -
                     Det3(bx, by, bz, cx, cy, cz, dx, dy, dz);

    double t = in_circle.FirstPositiveAscendingRoot();
    if (!(bool) isnan(t)) {
        auto edge_flip_event = make_shared<EdgeFlipEvent>(edge, triangulation, time_ + t);
        edge_flip_events.insert(std::pair<int,
                std::shared_ptr<EdgeFlipEvent>>(edge->id_, edge_flip_event));
        event_queue.insert(&*edge_flip_event);
    }
}


void Manager::RemoveEdgeFlipEvents(weak_ptr<Triangulation::Edge> &edge) {
    auto it = edge_flip_events.find(edge.lock()->id_);
    if (it != edge_flip_events.end()) {
        event_queue.erase(&*(it->second));
        edge_flip_events.erase(it);
    }
}

void Manager::Invalidate(shared_ptr<Triangulation::Vertex> &p) {
    RemoveReflectEvents(p);
    NewReflectInstance(p, true);
    NewReflectInstance(p, false);
    for (auto v2: triangulation->inner_vertices_) {
        if (p->point_->id_ < v2->point_->id_) {
            RemoveCollideEvents(p, v2);
            NewCollideInstance(p, v2);
        } else if (p->point_->id_ > v2->point_->id_) {
            RemoveCollideEvents(v2, p);
            NewCollideInstance(v2, p);
        }
    }

    //TODO(yue) delete the old events
    for (auto edge: triangulation->GetOutgoingEdges(p)) {
        std::vector<weak_ptr<Triangulation::Edge>> event_edge_list{
                edge, edge->pair_, edge->next_, edge->next_.lock()->pair_
        };
        for (auto event_edge : event_edge_list) {
            if (event_edge.expired()) {
                continue;
            }
            RemoveEdgeFlipEvents(event_edge);
        }
    }

    //TODO(yue) add new events
    for (auto edge: triangulation->GetOutgoingEdges(p)) {
        NewEdgeFlipInstance(edge);
        auto nextEdge = edge->next_.lock();
        NewEdgeFlipInstance(nextEdge);
    }
}

void Manager::InvalidateEdge(shared_ptr<Triangulation::Edge> &edge) {
    std::vector<weak_ptr<Triangulation::Edge>> edges_to_delete{
            edge, edge->pair_, edge->next_, edge->next_.lock()->pair_,
            edge->next_.lock()->next_, edge->next_.lock()->next_.lock()->pair_,
            edge->pair_.lock()->next_, edge->pair_.lock()->next_.lock()->pair_,
            edge->pair_.lock()->next_.lock()->next_.lock(), edge->pair_.lock()->next_.lock()->next_.lock()->pair_
    };

    for (auto event_edge: edges_to_delete) {
        if (event_edge.expired()) {
            continue;
        }
        RemoveEdgeFlipEvents(event_edge);
    }

    std::vector<weak_ptr<Triangulation::Edge>> edges_to_add{
            edge, edge->next_, edge->next_.lock()->next_, edge->pair_.lock()->next_,
            edge->pair_.lock()->next_.lock()->next_
    };

    for (auto event_edge: edges_to_add) {
        if (event_edge.expired()) {
            continue;
        }
        auto shared_edge = event_edge.lock();
        NewEdgeFlipInstance(shared_edge);
    }
}
