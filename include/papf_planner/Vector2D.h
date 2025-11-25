#pragma once
#include <cmath>

struct Vector2D {
    double x = 0.0, y = 0.0;

    Vector2D() = default;
    Vector2D(double x, double y) : x(x), y(y) {}

    double magnitude() const { return std::sqrt(x * x + y * y); }
    void normalize() {
        double mag = magnitude();
        if (mag > 1e-9) {
            x /= mag;
            y /= mag;
        }
    }
    Vector2D normalized() const {
        Vector2D vec = *this;
        vec.normalize();
        return vec;
    }
    double distanceTo(const Vector2D& other) const {
        return std::sqrt(std::pow(x - other.x, 2) + std::pow(y - other.y, 2));
    }
    double distanceToSquared(const Vector2D& other) const {
        return std::pow(std::sqrt(std::pow(x - other.x, 2) + std::pow(y - other.y, 2)),2);
    }

    Vector2D operator+(const Vector2D& other) const { return {x + other.x, y + other.y}; }
    Vector2D operator-(const Vector2D& other) const { return {x - other.x, y - other.y}; }
    Vector2D operator*(double scalar) const { return {x * scalar, y * scalar}; }
};