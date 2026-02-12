#pragma once

#include <vector>

struct Point {
    double x, y;
};

struct Box {
    Point c;
    double w, h;
    double yaw;
};

struct Edge {
    Point a, b;
};

Point vect(const Point &a, const Point &b);
double cross(const Point &u, const Point &v);
bool inside(const Edge &e, const Point &p);
bool ccw(const Point &a, const Point &b, const Point &c);
double dist(const Point &a, const Point &b);
double dot(const Point &a, const Point &b);
double proj_norm(const Point &p, const Point &v);

Point intersect(const Edge &e, const Edge &f);
Point rotatedPoint(const Point &p, const Point &offset, double yaw);
std::vector<Point> boxToPoints(const Box &b);

std::vector<Point> suthodg(const Box &a, const Box &b);
double polygonArea(const std::vector<Point> &polygon);
double boxIntersectArea(const Box &a, const Box &b);

std::vector<Point> convexHull(const std::vector<Point> &pts);
double minAreaBoxDiagSquared(const Box &a, const Box &b);
