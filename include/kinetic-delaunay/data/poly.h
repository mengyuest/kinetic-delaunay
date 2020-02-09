#ifndef KINETIC_DELAUNAY_POLY_H
#define KINETIC_DELAUNAY_POLY_H

#include <memory>
#include <vector>

class Poly {
public:
    std::vector<double> coefficients_;

    Poly(std::vector<double> coefficients);
    bool operator== (const Poly &rhs) const;
    double Eval(double t);
    Poly Derivative();
    std::vector<double> PositiveRoots();
    double FirstPositiveAscendingRoot();
};

int Signum(double x);
bool SignBit(double x);
Poly operator+(Poly const &a, Poly const &b);
Poly operator-(Poly const &a, Poly const &b);
Poly operator*(Poly const &a, Poly const &b);
Poly Det2(Poly const &a, Poly const &b, Poly const &c, Poly const &d);
Poly Det3(Poly const &a, Poly const &b, Poly const &c,
          Poly const &d, Poly const &e, Poly const &f,
          Poly const &g, Poly const &h, Poly const &i);
#endif //KINETIC_DELAUNAY_POLY_H
