#include "ByteTrack/iou_utils.h"

#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>

double EPS = 1e-6;

Point vect(const Point &a, const Point &b) {
    return Point{b.x - a.x, b.y - a.y};
}
double cross(const Point &u, const Point &v) {
    return u.x * v.y - u.y * v.x;
}

bool inside(const Edge &e, const Point &p) {
    return cross(vect(e.a, e.b), vect(e.a, p)) >= -EPS;
}

bool ccw(const Point &a, const Point &b, const Point &c) {
    return cross(vect(a, b), vect(a, c)) > -EPS;
}

double dist(const Point &a, const Point &b) {
    double dx = b.x - a.x, dy = b.y - a.y;
    return std::sqrt(dx*dx + dy*dy);
}

double dot(const Point &a, const Point &b) {
    return a.x * b.x + a.y * b.y;
}

double proj_norm(const Point &p, const Point &v) {
    return std::abs(dot(p, v)) / std::sqrt(dot(v, v));
}

Point intersect(const Edge &e, const Edge &f) {
    double edx = e.a.x - e.b.x, edy = e.a.y - e.b.y;
    double fdx = f.a.x - f.b.x, fdy = f.a.y - f.b.y;
    double e_crs = cross(e.a, e.b), f_crs = cross(f.a, f.b);
    double denom = edx*fdy - edy*fdx;
    return Point{
        (e_crs*fdx - edx*f_crs) / denom,
        (e_crs*fdy - edy*f_crs) / denom
    };
}

Point rotatedPoint(const Point &p, const Point &offset, double yaw) {
    double sin_th = std::sin(yaw), cos_th = std::cos(yaw);
    return Point{
        p.x*cos_th - p.y*sin_th + offset.x,
        p.x*sin_th + p.y*cos_th + offset.y
    };
}

std::vector<Point> boxToPoints(const Box &b) {
    return std::vector<Point>{
        rotatedPoint(Point{-b.w/2, -b.h/2}, b.c, b.yaw),
        rotatedPoint(Point{b.w/2, -b.h/2}, b.c, b.yaw),
        rotatedPoint(Point{b.w/2, b.h/2}, b.c, b.yaw),
        rotatedPoint(Point{-b.w/2, b.h/2}, b.c, b.yaw)
    };
}

std::vector<Point> suthodg(const Box &a, const Box &b) {
    std::vector<Point> pts_a = boxToPoints(a);
    std::vector<Point> pts_b = boxToPoints(b);
    std::vector<Point> temp;

    for (size_t i = 0; i < pts_a.size(); ++i) {
        Edge e = Edge{pts_a[i], pts_a[(i+1) % pts_a.size()]};
        for (size_t j = 0; j < pts_b.size(); ++j) {
            Edge f = Edge{pts_b[j], pts_b[(j+1) % pts_b.size()]};
            bool fa_in = inside(e, f.a), fb_in = inside(e, f.b);
            if (fb_in) {
                if (!fa_in) temp.push_back(intersect(e, f));
                temp.push_back(f.b);
            } else if (fa_in) {
                temp.push_back(intersect(e, f));
            }
        }
        pts_b = temp;
        temp.clear();
    }

    return pts_b;
}

double polygonArea(const std::vector<Point> &polygon) {
    if (polygon.size() <= 2) return 0.0f;
    double area = 0;
    for (size_t i = 0; i < polygon.size(); ++i) {
        Point curr = polygon[i], next = polygon[(i+1) % polygon.size()];
        area += cross(curr, next);
    }

    return std::abs(area) / 2.0f;
}

double boxIntersectArea(const Box &a, const Box &b) {
    if (a.w <= 0 || a.h <= 0 || b.w <= 0 || b.h <= 0) return 0.0f;
    return polygonArea(suthodg(a, b));
}

std::vector<Point> convexHull(std::vector<Point> &pts) {
    Point anchor;
    auto min_it = std::min_element(
        pts.begin(),
        pts.end(),
        [](const Point &p, const Point &q) {
            if (p.y != q.y) return p.y < q.y;
            else return p.x < q.x;
        }
    );

    if (min_it == pts.end()) {
        return std::vector<Point>{};
    } else {
        anchor = *min_it;
    }

    pts.erase(min_it);

    std::sort(
        pts.begin(),
        pts.end(),
        [&](Point p, Point q) {
            double c = cross(vect(anchor, p), vect(anchor, q));
            if (c != 0) return c > 0;
            else return dist(anchor, p) < dist(anchor, q);
        }
    );

    std::vector<Point> stack = {anchor};
    for (const auto &pt : pts) {
        while (stack.size() > 1 && !ccw(stack[stack.size() - 2], stack.back(), pt)) {
            stack.pop_back();
        }
        stack.push_back(pt);
    }

    return stack;
}

double minAreaBoxDiagSquared(const Box &a, const Box &b) {
    std::vector<Point> pts_a = boxToPoints(a);
    std::vector<Point> pts_b = boxToPoints(b);
    pts_a.insert(pts_a.end(), pts_b.begin(), pts_b.end());
    std::vector<Point> cvex_hull = convexHull(pts_a);

    // assumes ccw hull
    if (cvex_hull.size() <= 1) return 0.0f;
    if (cvex_hull.size() == 2) return dist(cvex_hull[0], cvex_hull[1]);

    double min_area = std::numeric_limits<double>::max();
    double min_box_diag = 0.0f;

    for (size_t i = 0; i < cvex_hull.size(); ++i) {
        Point curr = cvex_hull[i];
        Point next = cvex_hull[(i+1) % cvex_hull.size()];
        Point base = vect(curr, next);
        Point perp = {-base.y, base.x};

        auto max_base = *(std::max_element(
            cvex_hull.begin(), cvex_hull.end(),
            [&](const Point &p, const Point &q) {
                return dot(vect(curr, p), base) < dot(vect(curr, q), base);
            }
        ));
        auto max_perp = *(std::max_element(
            cvex_hull.begin(), cvex_hull.end(),
            [&](const Point &p, const Point &q) {
                return dot(vect(curr, p), perp) < dot(vect(curr, q), perp);
            }
        ));

        double w = proj_norm(vect(curr, max_base), base), h = proj_norm(vect(curr, max_perp), perp);
        double area = w * h;

        if (area < min_area) {
            min_area = area;
            min_box_diag = w*w + h*h;
        }
    }
    return min_box_diag;
}
