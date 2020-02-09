#include "gtest/gtest.h"
#include "kinetic-delaunay/data/triangulation.h"

TEST(Triangulation, TestForTriangulation) {
    auto a = make_shared<MyPoint>(0, 1);
    auto b = make_shared<MyPoint>(1, -1);
    auto c = make_shared<MyPoint>(2, 1);
    auto d = make_shared<MyPoint>(1, 3);
    double bound = 300;
    double factor = 10000;

    Triangulation triangulation(std::vector<shared_ptr<MyPoint>>{a,b,c,d}, bound, factor);
    EXPECT_EQ(triangulation.vertices_.size(), 7);
    EXPECT_EQ(triangulation.inner_vertices_.size(), 4);
    EXPECT_EQ(triangulation.boundary_vertices_.size(), 3);
    EXPECT_EQ(triangulation.edges_.size(), 27);
    for(auto vertex: triangulation.inner_vertices_){
        if (vertex->point_->id_==1){
            EXPECT_EQ(triangulation.GetOutgoingEdges(vertex).size(), 5);
            bool found_delaunay_edge=false;
            for(auto edge:triangulation.GetOutgoingEdges(vertex)){
                if (edge->next_.lock()->vertex_.lock()->point_->id_==3){
                    found_delaunay_edge=true;
                }
            }
            EXPECT_TRUE(found_delaunay_edge);
        }
    }
    EXPECT_EQ(triangulation.edges_.size(), 27);
}