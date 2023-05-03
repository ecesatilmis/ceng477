#ifndef __COLOR_H__
#define __COLOR_H__

#include <iostream>

class Color
{
public:
    double r, g, b;

    Color();
    Color(double r, double g, double b);
    Color(const Color &other);
    friend std::ostream& operator<<(std::ostream& os, const Color& c);
    Color operator-(const Color &rhs) const;
    Color operator/(const double value) const;
    Color operator*(const double value);
    Color operator+(const Color &rhs);
    void operator=(const Color &rhs);
    void operator+=(const Color &rhs);
    void operator-=(const Color &rhs);
};

#endif