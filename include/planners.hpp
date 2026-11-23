#pragma once
#include <queue>
#include <random>
#include <chrono>
#include "grid.hpp"

struct PlanResult { std::vector<Pt> path; double length = 0; int expanded = 0; double ms = 0; bool ok = false; std::vector<std::pair<Pt, Pt>> tree; };
using Clock = std::chrono::steady_clock;
inline double since_ms(Clock::time_point t0) { return std::chrono::duration<double, std::milli>(Clock::now() - t0).count(); }

// ---------------- A* (8-connected, octile heuristic) ----------------
inline PlanResult astar(const Grid& g, Pt s, Pt t) {
    auto t0 = Clock::now(); PlanResult R;
    int si = (int)(s.x / g.res), sj = (int)(s.y / g.res), ti = (int)(t.x / g.res), tj = (int)(t.y / g.res);
    if (!g.in(si, sj) || !g.in(ti, tj) || g.inf[g.idx(ti, tj)]) { R.ms = since_ms(t0); return R; }
    auto h = [&](int i, int j) { double dx = std::abs(i - ti), dy = std::abs(j - tj); return (dx + dy) + (1.4142 - 2) * std::min(dx, dy); };
    std::vector<double> gc(g.w * g.h, 1e18); std::vector<int> par(g.w * g.h, -1); std::vector<uint8_t> closed(g.w * g.h, 0);
    using QE = std::pair<double, int>; std::priority_queue<QE, std::vector<QE>, std::greater<QE>> open;
    gc[g.idx(si, sj)] = 0; open.push({h(si, sj), g.idx(si, sj)});
    while (!open.empty()) {
        int cur = open.top().second; open.pop();
        if (closed[cur]) continue;
        closed[cur] = 1; R.expanded++;
        int ci = cur % g.w, cj = cur / g.w;
        if (ci == ti && cj == tj) break;
        for (int dj = -1; dj <= 1; dj++) for (int di = -1; di <= 1; di++) {
            if (!di && !dj) continue;
            int ni = ci + di, nj = cj + dj;
            if (!g.in(ni, nj) || g.inf[g.idx(ni, nj)] || closed[g.idx(ni, nj)]) continue;
            if (di && dj && (g.inf[g.idx(ci + di, cj)] || g.inf[g.idx(ci, cj + dj)])) continue;     // no corner cutting
            double nc = gc[cur] + ((di && dj) ? 1.4142 : 1.0);
            if (nc < gc[g.idx(ni, nj)]) { gc[g.idx(ni, nj)] = nc; par[g.idx(ni, nj)] = cur; open.push({nc + h(ni, nj), g.idx(ni, nj)}); }
        }
    }
    if (par[g.idx(ti, tj)] < 0 && !(si == ti && sj == tj)) { R.ms = since_ms(t0); return R; }
    std::vector<Pt> p; for (int c = g.idx(ti, tj); c >= 0; c = par[c]) p.push_back({(c % g.w + .5) * g.res, (c / g.w + .5) * g.res});
    std::reverse(p.begin(), p.end()); p.front() = s; p.back() = t;
    R.path = smooth(g, p); R.length = path_length(R.path); R.ok = true; R.ms = since_ms(t0); return R;
}

// ---------------- RRT* (asymptotically optimal sampling planner) ----------------
inline PlanResult rrtstar(const Grid& g, Pt s, Pt t, int iters = 3000, unsigned seed = 1) {
    auto t0 = Clock::now(); PlanResult R; std::mt19937 rng(seed);
    std::uniform_real_distribution<double> ux(0, g.w * g.res), uy(0, g.h * g.res), u01(0, 1);
    struct N { Pt p; int parent; }; std::vector<N> T{{s, -1}};
    auto cost = [&](int i) { double c = 0; while (T[i].parent >= 0) { c += dist(T[i].p, T[T[i].parent].p); i = T[i].parent; } return c; };
    const double STEP = 2.5, GAMMA = 14.0;
    for (int it = 0; it < iters; it++) {
        Pt q = (u01(rng) < 0.05) ? t : Pt{ux(rng), uy(rng)};
        int near = 0; for (size_t i = 1; i < T.size(); i++) if (dist(T[i].p, q) < dist(T[near].p, q)) near = (int)i;
        double d = dist(T[near].p, q); if (d < 1e-9) continue;
        Pt nw{T[near].p.x + (q.x - T[near].p.x) * std::min(1.0, STEP / d), T[near].p.y + (q.y - T[near].p.y) * std::min(1.0, STEP / d)};
        if (g.blocked(nw.x, nw.y) || !g.line_free(T[near].p, nw)) continue;
        double rad = std::min(GAMMA * std::sqrt(std::log((double)T.size() + 1) / (T.size() + 1)), 6.0);
        std::vector<int> nb; for (size_t i = 0; i < T.size(); i++) if (dist(T[i].p, nw) < rad && g.line_free(T[i].p, nw)) nb.push_back((int)i);
        int best = near; double bc = cost(near) + dist(T[near].p, nw);
        for (int i : nb) { double c = cost(i) + dist(T[i].p, nw); if (c < bc) { bc = c; best = i; } }
        T.push_back({nw, best}); int ni = (int)T.size() - 1;
        for (int i : nb) if (i != best && bc + dist(T[i].p, nw) < cost(i)) T[i].parent = ni;      // rewire
    }
    int bi = -1; double bcst = 1e18;
    for (size_t i = 0; i < T.size(); i++) if (dist(T[i].p, t) < 3.0 && g.line_free(T[i].p, t)) { double c = cost((int)i) + dist(T[i].p, t); if (c < bcst) { bcst = c; bi = (int)i; } }
    R.expanded = (int)T.size();
    for (size_t i = 1; i < T.size(); i++) R.tree.push_back({T[i].p, T[T[i].parent].p});
    if (bi < 0) { R.ms = since_ms(t0); return R; }
    std::vector<Pt> p{t}; for (int i = bi; i >= 0; i = T[i].parent) p.push_back(T[i].p);
    std::reverse(p.begin(), p.end()); R.path = smooth(g, p); R.length = path_length(R.path); R.ok = true; R.ms = since_ms(t0); return R;
}
