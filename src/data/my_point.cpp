#include "kinetic-delaunay/data/my_point.h"

int MyPoint::num_points = 0;

MyPoint::MyPoint(double x, double y) : x_(x), y_(y), id_(++num_points) {}

MyPoint::MyPoint(double x, double y, double r) : x_(x), y_(y), r_(r), id_(++num_points) {}

MyPoint::MyPoint(double x, double y, double vx, double vy) : x_(x), y_(y), v_x_(vx), v_y_(vy), id_(++num_points) {}

MyPoint::MyPoint(double x, double y, double vx, double vy, double r) : x_(x), y_(y), v_x_(vx), v_y_(vy), r_(r),
                                                                       id_(++num_points) {}

using namespace Eigen;

const double MyPoint::kInfinitesimal = 1e-9;
const double MyPoint::kInfinity = 1e9;
double MyPoint::base_x_ = 0;
double MyPoint::base_y_ = 0;

double MyPoint::CounterClockWise(MyPoint a, MyPoint b, MyPoint c) {
    Matrix3d matrix;
    matrix << a.x_, a.y_, 1,
            b.x_, b.y_, 1,
            c.x_, c.y_, 1;
    return matrix.determinant();
}

bool MyPoint::InTriangle(MyPoint a, MyPoint b, MyPoint c) {
    return CounterClockWise(a, b, *this) >= 0 &&
           CounterClockWise(b, c, *this) >= 0 &&
           CounterClockWise(c, a, *this) >= 0;
}

double MyPoint::InCircle(MyPoint a, MyPoint b, MyPoint c) {
    Matrix4d matrix;
    matrix << a.x_, a.y_, a.x_ * a.x_ + a.y_ * a.y_, 1,
            b.x_, b.y_, b.x_ * b.x_ + b.y_ * b.y_, 1,
            c.x_, c.y_, c.x_ * c.x_ + c.y_ * c.y_, 1,
            this->x_, this->y_, this->x_ * this->x_ + this->y_ * this->y_, 1;
    double det = matrix.determinant();
    if (det == 0) {
        return 0;
    } else if (det > 0) {
        return 1;
    } else {
        return -1;
    }
}

bool MyPoint::operator<(const MyPoint &right) const {
    return this->id_ < right.id_;
}