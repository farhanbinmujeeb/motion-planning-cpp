#pragma once
#include "grid.hpp"

struct State { double x = 0, y = 0, th = 0, v = 0, w = 0; };
struct Cmd { double v = 0, w = 0; };
constexpr double ROBOT_R = 0.5, V_MAX = 2.0, W_MAX = 1.5, A_MAX = 1.5, ALPHA_MAX = 4.0;

inline double clampd(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }
inline double wrap_pi(double a) { while (a > M_PI) a -= 2 * M_PI; while (a < -M_PI) a += 2 * M_PI; return a; }

// Unicycle with acceleration limits (same model used for simulation and for MPC rollouts).
inline State step_state(State s, Cmd c, double dt) {
    s.v += clampd(c.v - s.v, -A_MAX * dt, A_MAX * dt); s.w += clampd(c.w - s.w, -ALPHA_MAX * dt, ALPHA_MAX * dt);
    s.th = wrap_pi(s.th + s.w * dt); s.x += s.v * std::cos(s.th) * dt; s.y += s.v * std::sin(s.th) * dt; return s;
}

// Moving obstacle with known ping-pong motion model (so the controller can predict it).
struct Dyn {
    Pt a, b; double speed, r;
    Pt at(double t) const { double L = dist(a, b), d = std::fmod(speed * t, 2 * L); if (d > L) d = 2 * L - d; return {a.x + (b.x - a.x) * d / L, a.y + (b.y - a.y) * d / L}; }
};

// Point `L` metres ahead of the robot's projection onto the polyline (advances `seg` monotonically).
inline Pt carrot(const std::vector<Pt>& p, Pt pos, double L, size_t& seg) {
    if (p.size() < 2) return p.empty() ? pos : p[0];
    double bd = 1e18; size_t bs = seg; Pt bp = p[seg];
    for (size_t k = seg; k < std::min(p.size() - 1, seg + 3); k++) {
        Pt a = p[k], b = p[k + 1]; double dx = b.x - a.x, dy = b.y - a.y, l2 = dx * dx + dy * dy;
        double u = l2 > 0 ? clampd(((pos.x - a.x) * dx + (pos.y - a.y) * dy) / l2, 0, 1) : 0; Pt q{a.x + u * dx, a.y + u * dy};
        if (dist(q, pos) < bd) { bd = dist(q, pos); bs = k; bp = q; }
    }
    seg = bs; double rem = L; Pt cur = bp;
    for (size_t k = seg; k + 1 < p.size(); k++) { double d = dist(cur, p[k + 1]); if (d >= rem) return {cur.x + (p[k + 1].x - cur.x) * rem / d, cur.y + (p[k + 1].y - cur.y) * rem / d}; rem -= d; cur = p[k + 1]; }
    return p.back();
}

// Sampling-based receding-horizon (MPC-style) controller: roll out candidate (v, w) commands over a
// 2.5 s horizon with the vehicle model, reject collisions (static map + predicted moving obstacles),
// score the rest by tracking error + clearance + speed, apply the best first command, repeat.
inline Cmd mpc(const Grid& g, const State& s, Pt ref, const std::vector<Dyn>& dyn, double t_now) {
    const double DT = 0.1; const int N = 25; double bestJ = 1e18; Cmd best{0, 0}; bool any = false;
    for (double v = 0; v <= V_MAX + 1e-9; v += 0.25) for (double w = -W_MAX; w <= W_MAX + 1e-9; w += 0.25) {
        State x = s; double J = 0; bool ok = true;
        for (int k = 1; k <= N && ok; k++) {
            x = step_state(x, {v, w}, DT);
            double c = g.clearance(x.x, x.y) - ROBOT_R; if (c <= 0) { ok = false; break; }
            if (c < 1.0) J += 3.0 * (1.0 - c) * DT;
            for (const auto& d : dyn) { double gap = dist({x.x, x.y}, d.at(t_now + k * DT)) - (d.r + ROBOT_R); if (gap <= 0.15) { ok = false; break; } if (gap < 1.5) J += 4.0 * (1.5 - gap) * DT; }
        }
        if (!ok) continue;
        double he = std::fabs(wrap_pi(std::atan2(ref.y - x.y, ref.x - x.x) - x.th));
        J += 2.0 * dist({x.x, x.y}, ref) + 0.5 * he - 0.3 * x.v;
        if (J < bestJ) { bestJ = J; best = {v, w}; any = true; }
    }
    return any ? best : Cmd{0, 0};                       // no safe trajectory: brake
}
