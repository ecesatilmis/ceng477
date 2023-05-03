#include "Color.h"
#include <iostream>
#include <iomanip>

using namespace std;

Color::Color() {}

Color::Color(double r, double g, double b)
{
    this->r = r;
    this->g = g;
    this->b = b;
}

Color::Color(const Color &other)
{
    this->r = other.r;
    this->g = other.g;
    this->b = other.b;
}

ostream& operator<<(ostream& os, const Color& c)
{
    os << fixed << setprecision(0) << "rgb(" << c.r << ", " << c.g << ", " << c.b << ")";
    return os;
}

Color Color::operator-(const Color &rhs) const{
    Color result;
    result.r = this->r - rhs.r;
    result.g = this->g - rhs.g;
    result.b = this->b - rhs.b;
    return result;
}

Color Color::operator/(const double value) const{
    Color result;
    result.r = this->r / value;
    result.g = this->g / value;
    result.b = this->b / value;
    return result;
}

void Color::operator+=(const Color &rhs){
    this->r = this->r + rhs.r;
    this->g = this->g + rhs.g;
    this->b = this->b + rhs.b;
}

void Color::operator-=(const Color &rhs){
    this->r = this->r - rhs.r;
    this->g = this->g - rhs.g;
    this->b = this->b - rhs.b;
}


void Color::operator=(const Color &rhs){
    this->r = rhs.r;
    this->g = rhs.g;
    this->b = rhs.b;
}

Color Color::operator+(const Color &rhs){
    return Color(this->r+rhs.r, this->g+rhs.g, this->b+rhs.b);
}

Color Color::operator*(const double d){
    return Color(this->r*d, this->g*d, this->b*d);
}
