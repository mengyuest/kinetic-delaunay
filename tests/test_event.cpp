#include "gtest/gtest.h"
#include "kinetic-delaunay/controller/my_event.h"
#include "kinetic-delaunay/controller/collide_event.h"
#include "kinetic-delaunay/controller/edge_flip_event.h"
#include "kinetic-delaunay/controller/reflect_event.h"
#include "kinetic-delaunay/data/triangulation.h"

TEST(MyEvent, TestForReflectEvent) {
    auto p0 = make_shared<MyPoint>(0,0,20,10);
    auto v0 = make_shared<Triangulation::Vertex>(p0);
    ReflectEvent a(v0, false, 1);
    ReflectEvent b(v0, true, 2);
    EXPECT_DOUBLE_EQ(v0->point_->v_x_, 20);
    EXPECT_DOUBLE_EQ(v0->point_->v_y_, 10);

    a.Process();
    EXPECT_DOUBLE_EQ(v0->point_->v_x_, 20);
    EXPECT_DOUBLE_EQ(v0->point_->v_y_, -10);

    b.Process();
    EXPECT_DOUBLE_EQ(v0->point_->v_x_, -20);
    EXPECT_DOUBLE_EQ(v0->point_->v_y_, -10);
}

TEST(MyEvent, TestForCollideEvent) {
    auto p0 = make_shared<MyPoint>(-2,1,20,-10);
    auto v0 = make_shared<Triangulation::Vertex>(p0);
    auto p1 = make_shared<MyPoint>(0,0,-20,10);
    auto v1 = make_shared<Triangulation::Vertex>(p1);
    auto p2 = make_shared<MyPoint>(4,-2,0,0);
    auto v2 = make_shared<Triangulation::Vertex>(p2);

    CollideEvent a(v0, v1, 3);
    a.Process();
    EXPECT_DOUBLE_EQ(v0->point_->v_x_, -20);
    EXPECT_DOUBLE_EQ(v0->point_->v_y_, 10);
    EXPECT_DOUBLE_EQ(v1->point_->v_x_, 20);
    EXPECT_DOUBLE_EQ(v1->point_->v_y_, -10);


    CollideEvent b(v1, v2, 4);
    b.Process();
    EXPECT_DOUBLE_EQ(v1->point_->v_x_, 0);
    EXPECT_DOUBLE_EQ(v1->point_->v_y_, 0);
    EXPECT_DOUBLE_EQ(v2->point_->v_x_, 20);
    EXPECT_DOUBLE_EQ(v2->point_->v_y_, -10);
}

TEST(MyEvent, TestForEdgeFlipEvent) {
    auto a = make_shared<MyPoint>(0, 1);
    auto b = make_shared<MyPoint>(1, -1);
    auto c = make_shared<MyPoint>(2, 1);
    auto d = make_shared<MyPoint>(1, 3);
    double bound = 300;
    double factor = 10000;

    auto triangulation = make_shared<Triangulation> (std::vector<shared_ptr<MyPoint>>{a,b,c,d}, bound, factor);

    for(auto vertex: triangulation->inner_vertices_){
        if (vertex->point_->id_==a->id_){
            for(auto edge:triangulation->GetOutgoingEdges(vertex)){
                if (edge->next_.lock()->vertex_.lock()->point_->id_==c->id_){
                    EdgeFlipEvent a(edge, triangulation, 5);
                    a.Process();
                    break;
                }
            }
        }
    }

    bool found_matched_edge = false;
    for(auto vertex: triangulation->inner_vertices_){
        if (vertex->point_->id_==b->id_){
            for(auto edge:triangulation->GetOutgoingEdges(vertex)){
                if (edge->next_.lock()->vertex_.lock()->point_->id_==d->id_){
                    found_matched_edge=true;
                    break;
                }
            }
        }
    }
    EXPECT_TRUE(found_matched_edge);
}