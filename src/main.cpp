// Motion planning & autonomy demo: A*, RRT*, sampling-based MPC controller, replanning mission executor.
#include <cstdio>
#include "planners.hpp"
#include "controller.hpp"
#include "svg.hpp"

static Grid make_map() {
    Grid g(100, 100, 0.5);                                          // 50 m x 50 m, 0.5 m cells
    g.border(0.5);
    g.rect(15, 0, 17, 32); g.rect(30, 18, 32, 50); g.rect(20, 38, 26, 44);
    g.rect(36, 8, 42, 14); g.rect(6, 30, 10, 36); g.rect(42, 28, 46, 34);
    g.build(ROBOT_R + 0.3);                                         // inflate by robot radius + 0.3 m margin
    return g;
}

enum class MState { PLAN, EXECUTE, DONE, FAILED };

int main() {
    Grid map = make_map();
    // ---------- Part 1: planner comparison ----------
    Pt S{3, 3}, G{44, 44};
    PlanResult A = astar(map, S, G), R = rrtstar(map, S, G, 3000, 1);
    std::printf("== Planner comparison: (3,3) -> (44,44) ==\n");
    std::printf("  A*    : %s  length=%.1f m  expanded=%d  time=%.1f ms\n", A.ok ? "ok" : "FAIL", A.length, A.expanded, A.ms);
    std::printf("  RRT*  : %s  length=%.1f m  nodes=%d  time=%.1f ms\n", R.ok ? "ok" : "FAIL", R.length, R.expanded, R.ms);
    { Svg s("planners.svg", map); for (auto& e : R.tree) s.line(e.first, e.second, "#aaa", 0.7, 0.7);
      s.path(R.path, "#e67e00", 3); s.path(A.path, "#1f5fd1", 3); s.dot(S, 0.6, "#0a0"); s.dot(G, 0.6, "#c00");
      s.text({1, 48.5}, "blue: A* | orange: RRT* | grey: RRT* tree"); }

    // ---------- Part 2: autonomous mission with a moving obstacle ----------
    std::vector<Pt> goals{{24, 25}, {44, 44}, {46, 5}};
    std::vector<Dyn> dyn{{{8, 12}, {8, 28}, 0.9, 0.8}, {{36, 24}, {44, 24}, 0.8, 0.8}};
    State s{S.x, S.y, M_PI / 4, 0, 0};
    std::vector<Pt> traj{{s.x, s.y}}, path; std::vector<std::vector<Pt>> dyn_trail(dyn.size());
    size_t gi = 0, seg = 0; MState ms = MState::PLAN;
    double t = 0, dist_travelled = 0, min_clear = 1e9, last_replan = -10, slow_since = -1; int replans = 0, collisions = 0, plan_fail = 0;
    const double DT = 0.1, T_MAX = 400;

    auto plan = [&](bool with_dyn) {
        Grid g = map;
        if (with_dyn) for (auto& d : dyn) g.stamp(d.at(t), d.r + 1.0);
        PlanResult p = astar(g, {s.x, s.y}, goals[gi]);
        if (p.ok) { path = p.path; seg = 0; } else plan_fail++;
        last_replan = t; return p.ok;
    };

    while (t < T_MAX && ms != MState::DONE && ms != MState::FAILED) {
        if (ms == MState::PLAN) { if (!plan(false)) { ms = MState::FAILED; break; } ms = MState::EXECUTE; }
        // --- replanning triggers: moving obstacle near upcoming path, or robot stalled ---
        bool trigger = false;
        if (t - last_replan > 4.0) {
            for (auto& d : dyn) { Pt dp = d.at(t); double md = 1e18;
                for (size_t k = seg; k + 1 < path.size(); k++) for (int q = 0; q <= 10; q++) { Pt a{path[k].x + (path[k + 1].x - path[k].x) * q / 10.0, path[k].y + (path[k + 1].y - path[k].y) * q / 10.0}; md = std::min(md, dist(a, dp)); }
                if (md < d.r + ROBOT_R + 0.6 && dist({s.x, s.y}, dp) < 7.0) trigger = true; }
            if (s.v < 0.15) { if (slow_since < 0) slow_since = t; if (t - slow_since > 3.0) trigger = true; } else slow_since = -1;
        }
        if (trigger) { if (plan(true)) replans++; slow_since = -1; }

        Pt ref = carrot(path, {s.x, s.y}, 3.0, seg);
        std::vector<Dyn> dd = dyn;
        Cmd c = mpc(map, s, ref, dd, t);
        State ns = step_state(s, c, DT); dist_travelled += dist({s.x, s.y}, {ns.x, ns.y}); s = ns; t += DT;
        double cl = map.clearance(s.x, s.y) - ROBOT_R; min_clear = std::min(min_clear, cl);
        for (auto& d : dyn) if (dist({s.x, s.y}, d.at(t)) < d.r + ROBOT_R) collisions++;
        if (cl <= 0) collisions++;
        if (traj.size() < 100000 && (int)(t / DT) % 5 == 0) { traj.push_back({s.x, s.y}); for (size_t i = 0; i < dyn.size(); i++) dyn_trail[i].push_back(dyn[i].at(t)); }
        if (dist({s.x, s.y}, goals[gi]) < 0.8) { std::printf("  reached goal %zu at t=%.1f s\n", gi + 1, t); if (++gi == goals.size()) ms = MState::DONE; else { ms = MState::PLAN; slow_since = -1; } }
    }
    std::printf("\n== Mission (A* global planner + sampling-MPC local controller, 2 moving obstacles) ==\n");
    std::printf("  result          : %s\n", ms == MState::DONE ? "ALL GOALS REACHED" : "NOT COMPLETED");
    std::printf("  mission time    : %.1f s   distance: %.1f m\n", t, dist_travelled);
    std::printf("  replans         : %d (failed: %d)\n", replans, plan_fail);
    std::printf("  collisions      : %d   min clearance to static obstacles: %.2f m\n", collisions, min_clear);
    { Svg v("mission.svg", map);
      for (size_t i = 0; i < dyn.size(); i++) for (auto& p : dyn_trail[i]) v.dot(p, 0.8, "#c00", 0.05);
      v.path(traj, "#0a8f3c", 2.5); v.dot(S, 0.6, "#0a0");
      for (size_t i = 0; i < goals.size(); i++) { v.dot(goals[i], 0.7, "#c00", 0.8); char b[32]; std::snprintf(b, sizeof b, "G%d", (int)i + 1); v.text({goals[i].x + 1, goals[i].y + 1}, b); }
      v.text({1, 48.5}, "green: executed trajectory | red haze: moving obstacles"); }
    std::printf("Wrote planners.svg and mission.svg\n");
    return ms == MState::DONE && collisions == 0 ? 0 : 1;
}
