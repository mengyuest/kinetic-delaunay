#include "kinetic-delaunay/data/poly.h"
#include "math.h"
#include "kinetic-delaunay/data/my_point.h"

Poly::Poly(std::vector<double> coefficients) : coefficients_(coefficients) {}

int Signum(double x) {
    if (x > 0.0) return 1;
    if (x < 0.0) return -1;
    return 0;
}

bool SignBit(double x){
    if (x>=0) return true;
    return false;
}

bool Poly::operator== (const Poly &rhs) const{
    if(this->coefficients_.size()!=rhs.coefficients_.size()){
        return false;
    }

    for (int i=0;i<this->coefficients_.size();i++){
        if (this->coefficients_[i]!=rhs.coefficients_[i]){
            return false;
        }
    }

    return true;
}

Poly operator+(Poly const &a, Poly const &b) {
    std::vector<double> coefficients(std::max(a.coefficients_.size(), b.coefficients_.size()), 0);
    for (int i = 0; i < a.coefficients_.size(); i++) {
        coefficients[i] += a.coefficients_[i];
    }
    for (int i = 0; i < b.coefficients_.size(); i++) {
        coefficients[i] += b.coefficients_[i];
    }
    return Poly(coefficients);
}

Poly operator-(Poly const &a, Poly const &b) {
    std::vector<double> coefficients(std::max(a.coefficients_.size(), b.coefficients_.size()), 0);
    for (int i = 0; i < a.coefficients_.size(); i++) {
        coefficients[i] += a.coefficients_[i];
    }
    for (int i = 0; i < b.coefficients_.size(); i++) {
        coefficients[i] -= b.coefficients_[i];
    }
    return Poly(coefficients);
}

Poly operator*(Poly const &a, Poly const &b) {
    std::vector<double> coefficients(a.coefficients_.size() + b.coefficients_.size() - 1, 0);
    for (int i = 0; i < a.coefficients_.size(); i++) {
        for (int j = 0; j < b.coefficients_.size(); j++) {
            coefficients[i + j] += a.coefficients_[i] * b.coefficients_[j];
        }
    }
    return Poly(coefficients);
}

Poly Det2(Poly const &a, Poly const &b, Poly const &c, Poly const &d) {
    return a * d - b * c;
}

Poly Det3(Poly const &a, Poly const &b, Poly const &c,
          Poly const &d, Poly const &e, Poly const &f,
          Poly const &g, Poly const &h, Poly const &i) {
    return (a * Det2(e, f, h, i)) - (b * Det2(d, f, g, i)) + (c * Det2(d, e, g, h));
}

double Poly::Eval(double t) {
    double answer = 0;
    for (int i = coefficients_.size() - 1; i >= 0; i--) {
        answer *= t;
        answer += coefficients_[i];
    }
    return answer;
}

Poly Poly::Derivative() {
    if (coefficients_.size() == 1)
        return Poly(std::vector<double>(1, 0));
    std::vector<double> new_coefficients(coefficients_.size() - 1, 0);
    for (int i = 0; i < new_coefficients.size(); i++) {
        new_coefficients[i] = (i + 1) * coefficients_[i + 1];
    }
    return Poly(new_coefficients);
}

std::vector<double> Poly::PositiveRoots() {
    std::vector<double> answer;
    int degree = coefficients_.size() - 1;
    while (degree > 0 and fabs(coefficients_[degree]) < MyPoint::kInfinitesimal) {
        degree--;
    }
    if (degree == 0)
        return answer;
    if (degree == 1) {
        double root = -coefficients_[0] / coefficients_[1];
        if (root > 0)
            answer.push_back(root);
        return answer;
    }

    std::vector<double> stubs = Derivative().PositiveRoots();
    stubs.insert(stubs.begin(), 0.0);
    double upper_bound = 2 * stubs[stubs.size() - 1] + 1;
    while (SignBit(Eval(upper_bound)) != SignBit(coefficients_[degree])) {
        upper_bound *= 2;
    }
    stubs.push_back(upper_bound);
    for (int i = 1; i < stubs.size(); i++) {
        double leftSign = Signum(Eval(stubs[i - 1]));
        double rightSign = Signum(Eval(stubs[i]));
        if (leftSign == rightSign)
            continue;

        double low = stubs[i - 1];
        double high = stubs[i];

        if (low > MyPoint::kInfinity) {
            continue;
        }

        while (std::nextafter(low, low + 1) < high) {
            double mid = (low + high) / 2;
            if (Signum(Eval(mid)) == leftSign) {
                low = mid;
            } else {
                high = mid;
            }
        }
        answer.push_back(low);
    }
    return answer;
}

double Poly::FirstPositiveAscendingRoot() {
    if (fabs(coefficients_[0]) < MyPoint::kInfinitesimal and coefficients_[1] > 0) {
        return 0;
    }

    std::vector<double> roots = PositiveRoots();
    int cursor = 0;
    Poly derivative = Derivative();
    while (cursor < roots.size() && derivative.Eval(roots[cursor]) <= 0) {
        cursor++;
    }
    return cursor < roots.size() ? roots[cursor] : NAN;
}
