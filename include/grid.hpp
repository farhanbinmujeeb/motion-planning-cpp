#pragma once
#include <vector>
#include <cmath>
#include <cstdint>
#include <algorithm>

struct Pt { double x = 0, y = 0; };
inline double dist(Pt a, Pt b) { return std::hypot(a.x - b.x, a.y - b.y); }

// Occupancy grid with obstacle-distance field and robot-radius inflation.
struct Grid {
    int w, h; double res;
    std::vector<uint8_t> occ, inf; std::vector<float> dst;
    Grid(int w_, int h_, double r) : w(w_), h(h_), res(r), occ(w_ * h_, 0), inf(w_ * h_, 0), dst(w_ * h_, 0) {}
    bool in(int i, int j) const { return i >= 0 && j >= 0 && i < w && j < h; }
    int idx(int i, int j) const { return j * w + i; }
    void rect(double x0, double y0, double x1, double y1) {
        for (int j = (int)(y0 / res); j < (int)std::ceil(y1 / res); j++)
            for (int i = (int)(x0 / res); i < (int)std::ceil(x1 / res); i++) if (in(i, j)) occ[idx(i, j)] = 1;
    }
    void border(double t) { rect(0, 0, w * res, t); rect(0, h * res - t, w * res, h * res); rect(0, 0, t, h * res); rect(w * res - t, 0, w * res, h * res); }

    // Two-pass chamfer distance transform, then inflate by robot radius.
    void build(double radius) {
        const float INF = 1e9f, a = 1.f, b = 1.4142f;
        for (int k = 0; k < w * h; k++) dst[k] = occ[k] ? 0.f : INF;
        for (int j = 0; j < h; j++) for (int i = 0; i < w; i++) { float& d = dst[idx(i, j)];
            if (in(i - 1, j)) d = std::min(d, dst[idx(i - 1, j)] + a);
            if (in(i, j - 1)) d = std::min(d, dst[idx(i, j - 1)] + a);
            if (in(i - 1, j - 1)) d = std::min(d, dst[idx(i - 1, j - 1)] + b);
            if (in(i + 1, j - 1)) d = std::min(d, dst[idx(i + 1, j - 1)] + b); }
        for (int j = h - 1; j >= 0; j--) for (int i = w - 1; i >= 0; i--) { float& d = dst[idx(i, j)];
            if (in(i + 1, j)) d = std::min(d, dst[idx(i + 1, j)] + a);
            if (in(i, j + 1)) d = std::min(d, dst[idx(i, j + 1)] + a);
            if (in(i + 1, j + 1)) d = std::min(d, dst[idx(i + 1, j + 1)] + b);
            if (in(i - 1, j + 1)) d = std::min(d, dst[idx(i - 1, j + 1)] + b); }
        for (int k = 0; k < w * h; k++) { dst[k] *= (float)res; inf[k] = (dst[k] - res / 2 < radius) ? 1 : 0; }
    }
    // Distance (m) from point to nearest obstacle edge; 0 outside the map.
    double clearance(double x, double y) const {
        int i = (int)(x / res), j = (int)(y / res); if (!in(i, j)) return 0.0;
        return std::max(0.0, (double)dst[idx(i, j)] - res / 2);
    }
    bool blocked(double x, double y) const { int i = (int)(x / res), j = (int)(y / res); return !in(i, j) || inf[idx(i, j)]; }
    bool line_free(Pt a, Pt b) const {
        int n = std::max(1, (int)(dist(a, b) / (res / 2)));
        for (int k = 0; k <= n; k++) { double t = (double)k / n; if (blocked(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t)) return false; }
        return true;
    }
    void stamp(Pt c, double r) {   // temporarily block a circular region (dynamic obstacle) in the planning map
        for (int j = 0; j < h; j++) for (int i = 0; i < w; i++)
            if (std::hypot((i + .5) * res - c.x, (j + .5) * res - c.y) < r) inf[idx(i, j)] = 1;
    }
};

inline double path_length(const std::vector<Pt>& p) { double s = 0; for (size_t i = 1; i < p.size(); i++) s += dist(p[i - 1], p[i]); return s; }

// Greedy line-of-sight shortcutting.
inline std::vector<Pt> smooth(const Grid& g, const std::vector<Pt>& p) {
    if (p.size() < 3) return p;
    std::vector<Pt> out{p[0]}; size_t i = 0;
    while (i < p.size() - 1) { size_t j = p.size() - 1; while (j > i + 1 && !g.line_free(p[i], p[j])) j--; out.push_back(p[j]); i = j; }
    return out;
}
