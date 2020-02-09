#include "gtest/gtest.h"
#include "kinetic-delaunay/data/my_point.h"

TEST(MyPoint, TestInTriangle) {
    MyPoint a(1, 1);
    MyPoint b(4, 1);
    MyPoint c(2, 2);
    MyPoint d(3, 2);
    MyPoint e(2, 3);
    MyPoint f(3, 1);

    EXPECT_EQ(c.InTriangle(a, d, e), true);
    EXPECT_EQ(c.InTriangle(a, e, d), false);
    EXPECT_EQ(b.InTriangle(a, d, e), false);
    EXPECT_EQ(f.InTriangle(a, d, e), false);
}

TEST(MyPoint, TestInCircle) {
    MyPoint a(1, 1);
    MyPoint b(4, 1);
    MyPoint c(2, 2);
    MyPoint d(3, 2);
    MyPoint e(2, 3);
    MyPoint f(3, 1);

    EXPECT_EQ(c.InCircle(a, d, e), 1);
    EXPECT_EQ(d.InCircle(a, d, e), 0);
    EXPECT_EQ(f.InCircle(a, d, e), -1);
    EXPECT_EQ(f.InCircle(a, e, d), 1);
}

TEST(MyPoint, TestLessThanOperator) {
    MyPoint a(1, 1);
    MyPoint b(4, 1);

    EXPECT_EQ(a<b, true);
    EXPECT_EQ(b<a, false);
}