#include "gtest/gtest.h"
#include "kinetic-delaunay/data/poly.h"

TEST(Poly, TestForSignum) {
    EXPECT_EQ(Signum(42), 1);
    EXPECT_EQ(Signum(0), 0);
    EXPECT_EQ(Signum(-42), -1);
}

TEST(Poly, TestForSignBit) {
    EXPECT_EQ(SignBit(42), true);
    EXPECT_EQ(SignBit(0), true);
    EXPECT_EQ(SignBit(-42), false);
}

TEST(Poly, TestForEqualOperator)
{
    Poly a(std::vector<double>{1,2,3});
    Poly b(std::vector<double>{1,2,3});
    Poly c(std::vector<double>{1,2,4});
    Poly d(std::vector<double>{1,2,3,4});
    EXPECT_EQ(a==b, true);
    EXPECT_EQ(a==c, false);
    EXPECT_EQ(a==d, false);
}

TEST(Poly, TestForAddOperator)
{
    Poly a(std::vector<double>{1,2,3});
    Poly b(std::vector<double>{4,5,7});
    EXPECT_TRUE(a+b==Poly(std::vector<double>{5,7,10}));
}

TEST(Poly, TestForMinusOperator)
{
    Poly a(std::vector<double>{1,2,3});
    Poly b(std::vector<double>{4,5,7});
    EXPECT_TRUE(b-a==Poly(std::vector<double>{3,3,4}));
    EXPECT_TRUE(a-a==Poly(std::vector<double>{0,0,0}));
}

TEST(Poly, TestForMulOperator)
{
    Poly a(std::vector<double>{1,2});
    Poly b(std::vector<double>{4,5});
    EXPECT_TRUE(a*b==Poly(std::vector<double>{4,13,10}));
    EXPECT_TRUE(b*a==a*b);
}

TEST(Poly, TestForDet2Operator)
{
    Poly a(std::vector<double>{1,2});
    Poly b(std::vector<double>{2,3});
    Poly c(std::vector<double>{3,4});
    Poly d(std::vector<double>{4,5});
    EXPECT_TRUE(Det2(a,b,c,d)==Poly(std::vector<double>{-2,-4,-2}));
}

TEST(Poly, TestForDet3Operator)
{
    Poly a(std::vector<double>{1,2});
    Poly b(std::vector<double>{0,0});
    Poly c(std::vector<double>{0,0});
    Poly d(std::vector<double>{4,5});
    Poly e(std::vector<double>{1,2});
    Poly f(std::vector<double>{0,0});
    Poly g(std::vector<double>{3,4});
    Poly h(std::vector<double>{1,5});
    Poly i(std::vector<double>{3,5});
    EXPECT_TRUE(Det3(a,b,c,d,e,f,g,h,i)==Poly(std::vector<double>{3,17,32,20}));
}

TEST(Poly, TestForEval)
{
    Poly a(std::vector<double>{3,4,1});
    double x=0;
    double y=2;
    EXPECT_DOUBLE_EQ(a.Eval(x), 3);
    EXPECT_DOUBLE_EQ(a.Eval(2), 15);
}

TEST(Poly, TestForDerivative)
{
    Poly a(std::vector<double>{1,2,3});
    EXPECT_TRUE(a.Derivative()==Poly(std::vector<double>{2,6}));
    EXPECT_TRUE(a.Derivative().Derivative()==Poly(std::vector<double>{6}));
    EXPECT_TRUE(a.Derivative().Derivative().Derivative()==Poly(std::vector<double>{0}));
}

TEST(Poly, TestForPositiveRoots)
{
    Poly a(std::vector<double>{-4,-3,1});
    Poly b(std::vector<double>{3,-4,1});
    Poly c(std::vector<double>{0,-3,1});
    Poly d(std::vector<double>{4,4,1});
    EXPECT_DOUBLE_EQ(a.PositiveRoots()[0], 4);
    EXPECT_DOUBLE_EQ(b.PositiveRoots()[0], 1);
    EXPECT_DOUBLE_EQ(b.PositiveRoots()[1], 3);
    EXPECT_DOUBLE_EQ(c.PositiveRoots()[0], 0);
    EXPECT_DOUBLE_EQ(c.PositiveRoots()[1], 3);
    EXPECT_EQ(d.PositiveRoots().size(), 0);
}

TEST(Poly, TestForFirstPositiveAscendingRoot)
{
    Poly a(std::vector<double>{2,-1,-2,1});
    EXPECT_DOUBLE_EQ(a.FirstPositiveAscendingRoot(), 2);
}