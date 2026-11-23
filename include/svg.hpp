#pragma once
#include <cstdio>
#include "planners.hpp"

struct Svg {
    FILE* f; double sc; Svg(const char* name, const Grid& g, double scale = 10) : f(std::fopen(name, "w")), sc(scale) {
        std::fprintf(f, "<svg xmlns='http://www.w3.org/2000/svg' width='%d' height='%d' style='background:#fff'>\n", (int)(g.w * g.res * sc), (int)(g.h * g.res * sc));
        for (int j = 0; j < g.h; j++) for (int i = 0; i < g.w; i++) if (g.occ[g.idx(i, j)])
            std::fprintf(f, "<rect x='%.1f' y='%.1f' width='%.1f' height='%.1f' fill='#444'/>\n", i * g.res * sc, (g.h - 1 - j) * g.res * sc, g.res * sc, g.res * sc);
        H = g.h * g.res;
    }
    double H;
    double Y(double y) const { return (H - y) * sc; }
    void line(Pt a, Pt b, const char* col, double w = 1, double op = 1) { std::fprintf(f, "<line x1='%.1f' y1='%.1f' x2='%.1f' y2='%.1f' stroke='%s' stroke-width='%.1f' opacity='%.2f'/>\n", a.x * sc, Y(a.y), b.x * sc, Y(b.y), col, w, op); }
    void path(const std::vector<Pt>& p, const char* col, double w = 2) { for (size_t i = 1; i < p.size(); i++) line(p[i - 1], p[i], col, w); }
    void dot(Pt c, double r, const char* col, double op = 1) { std::fprintf(f, "<circle cx='%.1f' cy='%.1f' r='%.1f' fill='%s' opacity='%.2f'/>\n", c.x * sc, Y(c.y), r * sc, col, op); }
    void text(Pt c, const char* s) { std::fprintf(f, "<text x='%.1f' y='%.1f' font-size='13' font-family='sans-serif' fill='#000'>%s</text>\n", c.x * sc, Y(c.y), s); }
    ~Svg() { std::fprintf(f, "</svg>\n"); std::fclose(f); }
};
