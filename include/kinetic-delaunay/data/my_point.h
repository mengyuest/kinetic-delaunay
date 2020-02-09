#ifndef KINETIC_DELAUNAY_MY_POINT_H
#define KINETIC_DELAUNAY_MY_POINT_H

#include <Eigen/Dense>
#include <memory>

class MyPoint {
public:
    static int num_points;
    int id_;
    double x_, y_;
    double v_x_, v_y_;
    double r_;

    const static double kInfinitesimal;
    const static double kInfinity;
    static double base_x_;
    static double base_y_;

    MyPoint(double x, double y);

    MyPoint(double x, double y, double r);

    MyPoint(double x, double y, double vx, double vy);

    MyPoint(double x, double y, double vx, double vy, double r);

    bool InTriangle(MyPoint a, MyPoint b, MyPoint c);

    double InCircle(MyPoint a, MyPoint b, MyPoint c);

    bool operator<(const MyPoint &right) const;

private:
    static double CounterClockWise(MyPoint a, MyPoint b, MyPoint c);
};

#endif //KINETIC_DELAUNAY_MY_POINT_H
