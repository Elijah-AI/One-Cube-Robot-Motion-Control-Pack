#ifndef MOTION_CONTROL_PKG_MOTION_JERK_LIMITED_RETIMING_HPP_
#define MOTION_CONTROL_PKG_MOTION_JERK_LIMITED_RETIMING_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <string>
#include <vector>

#include "motion_control_pkg/motion/trajectory_derivative_limits.hpp"

namespace motion_control_pkg::motion
{
struct RetimingResult
{
  bool valid{false};
  std::string error;
  double source_duration{0.0};
  double duration{0.0};
  double endpoint_window{0.0};
  double time_scale{1.0};
  std::vector<std::vector<double>> positions;
};

namespace retiming_detail
{
// Monotone change of time. Integrating smoothstep velocity gives a quartic
// ramp: f(u)=u^3-u^4/2. Only the two ends are rounded; the middle has unit
// progress speed. At rest, both progress velocity and acceleration are zero.
struct Clock
{
  double source_duration;
  double window;
  double duration() const {return source_duration + window;}
  std::array<double, 4> at(double t) const
  {
    t = std::clamp(t, 0.0, duration());
    const bool ending = t > duration() - window;
    const double local = ending ? duration() - t : t;
    if (local >= window) {return {t - window / 2, 1.0, 0.0, 0.0};}
    const double u = local / window;
    const double s = window * (u*u*u - 0.5*u*u*u*u);
    return {ending ? source_duration - s : s,
      3*u*u - 2*u*u*u,
      (ending ? -1 : 1) * 6*u*(1-u)/window,
      (6 - 12*u)/(window*window)};
  }
  double inverse(double s) const
  {
    double lo = 0, hi = duration();
    for (int i = 0; i < 60; ++i) {
      const double mid = (lo + hi) / 2;
      if (at(mid)[0] < s) {lo = mid;} else {hi = mid;}
    }
    return (lo + hi) / 2;
  }
  std::array<double, 3> derivativeBounds(double a, double b) const
  {
    std::array<double, 3> bounds{};
    // Each derivative is monotone between these critical points.
    for (double t : {a, b, window/2, window, duration()-window,
        duration()-window/2}) {
      if (t < a || t > b) {continue;}
      const auto state = at(t);
      for (double side : {std::nextafter(t, a), std::nextafter(t, b)}) {
        const auto edge = at(side);
        for (int k = 0; k < 3; ++k) {bounds[k] = std::max(bounds[k], std::abs(edge[k+1]));}
      }
      for (int k = 0; k < 3; ++k) {bounds[k] = std::max(bounds[k], std::abs(state[k+1]));}
    }
    return bounds;
  }
};

struct CubicSpan
{
  double begin, end, middle;
  std::vector<double> v, a, j;
  std::array<double, 3> derivativeBounds(std::size_t joint, double lo, double hi) const
  {
    lo -= middle; hi -= middle;
    auto velocity = [&](double x) {return v[joint] + a[joint]*x + 0.5*j[joint]*x*x;};
    const double a0 = a[joint] + j[joint]*lo, a1 = a[joint] + j[joint]*hi;
    double vb = std::max(std::abs(velocity(lo)), std::abs(velocity(hi)));
    if (std::abs(j[joint]) > 1e-15) {
      const double root = -a[joint]/j[joint];
      if (root > lo && root < hi) {vb = std::max(vb, std::abs(velocity(root)));}
    }
    return {vb, std::max(std::abs(a0), std::abs(a1)), std::abs(j[joint])};
  }
};
}  // namespace retiming_detail

// The evaluator MUST describe a C2 piecewise cubic with the supplied knots.
// Bounds are analytic over whole intervals, including between 4 ms samples.
// Chain rule: q''' = p''' s'^3 + 3 p'' s' s'' + p' s'''.
// The geometry p(s) is unchanged, unlike fitting a new joint-space smoother.
inline RetimingResult retimeCubicTrajectory(
  const std::vector<double>& knots,
  const std::function<std::vector<double>(double, int)>& evaluate,
  double dt, const std::vector<double>& velocity,
  const std::vector<double>& acceleration, const std::vector<double>& jerk,
  double maximum_duration = std::numeric_limits<double>::infinity())
{
  RetimingResult result;
  const std::size_t dof = velocity.size();
  auto fail = [&](const std::string& error) {result.error = error; return result;};
  if (knots.size() < 2 || dof == 0 || acceleration.size() != dof || jerk.size() != dof ||
    !std::isfinite(dt) || dt <= 0 || std::isnan(maximum_duration) || maximum_duration <= 0 ||
    !std::isfinite(knots.front()) || std::abs(knots.front()) > 1e-10) {
    return fail("invalid retiming input dimensions, period or time origin");
  }
  for (std::size_t j = 0; j < dof; ++j) {
    if (!std::isfinite(velocity[j]) || velocity[j] <= 0 ||
      !std::isfinite(acceleration[j]) || acceleration[j] <= 0 ||
      !std::isfinite(jerk[j]) || jerk[j] <= 0) {return fail("invalid retiming v/a/jerk limit");}
  }
  auto finite = [&](const std::vector<double>& x) {
    return x.size() == dof && std::all_of(x.begin(), x.end(), [](double y) {return std::isfinite(y);});
  };
  std::vector<retiming_detail::CubicSpan> spans;
  for (std::size_t i = 1; i < knots.size(); ++i) {
    const double lo = knots[i-1], hi = knots[i], h = hi-lo;
    if (!std::isfinite(hi) || h <= 1e-12) {return fail("non-increasing cubic time knots");}
    const double mid = (lo+hi)/2;
    const auto v = evaluate(mid, 1), a = evaluate(mid, 2);
    const auto a0 = evaluate(lo+h/4, 2), a1 = evaluate(hi-h/4, 2);
    if (!finite(v) || !finite(a) || !finite(a0) || !finite(a1)) {
      return fail("non-finite cubic derivatives");
    }
    std::vector<double> j(dof);
    for (std::size_t k = 0; k < dof; ++k) {j[k] = (a1[k]-a0[k])/(h/2);}
    if (!finite(j)) {return fail("non-finite cubic jerk");}
    spans.push_back({lo, hi, mid, v, a, j});
  }
  result.source_duration = knots.back();
  double best_duration = std::numeric_limits<double>::infinity();
  double best_window = 0;
  // Compare several local endpoint windows; do not assume a single fixed
  // ramp is suitable for both a short move and a long transport motion.
  for (double candidate : {0.04, 0.08, 0.16, 0.32, 0.64, result.source_duration/4}) {
    const double window = std::min(candidate, result.source_duration);
    retiming_detail::Clock clock{result.source_duration, window};
    double scale = 1.0;
    for (const auto& span : spans) {
      const double begin = clock.inverse(span.begin), end = clock.inverse(span.end);
      // Subdivision tightens conservative product bounds; correctness does
      // not depend on the subdivision density.
      for (int n = 0; n < 8; ++n) {
        const double t0 = begin + (end-begin)*n/8;
        const double t1 = begin + (end-begin)*(n+1)/8;
        const auto w = clock.derivativeBounds(t0, t1);
        const double s0 = clock.at(t0)[0], s1 = clock.at(t1)[0];
        for (std::size_t j = 0; j < dof; ++j) {
          const auto p = span.derivativeBounds(j, s0, s1);
          const double vb = p[0]*w[0];
          const double ab = p[1]*w[0]*w[0] + p[0]*w[1];
          const double jb = p[2]*w[0]*w[0]*w[0] + 3*p[1]*w[0]*w[1] + p[0]*w[2];
          if (!std::isfinite(vb) || !std::isfinite(ab) || !std::isfinite(jb)) {
            return fail("non-finite continuous derivative bound");
          }
          scale = std::max({scale, vb/velocity[j], std::sqrt(ab/acceleration[j]),
            std::cbrt(jb/jerk[j])});
        }
      }
    }
    // Numerical margin, followed by an exact integer number of command ticks.
    const double duration = std::ceil(clock.duration()*scale*(1+1e-6)/dt)*dt;
    if (duration < best_duration) {best_duration = duration; best_window = window;}
  }
  if (!std::isfinite(best_duration) || best_duration > maximum_duration ||
    best_duration/dt > 1000000) {return fail("jerk-limited duration exceeds timeout or sample budget");}
  retiming_detail::Clock clock{result.source_duration, best_window};
  result.duration = best_duration;
  result.endpoint_window = best_window;
  result.time_scale = best_duration/clock.duration();
  const auto steps = static_cast<std::size_t>(std::llround(best_duration/dt));
  result.positions.reserve(steps+1);
  for (std::size_t i = 0; i <= steps; ++i) {
    const double source_time = i == steps ? result.source_duration :
      clock.at(static_cast<double>(i)*dt/result.time_scale)[0];
    auto q = evaluate(source_time, 0);
    if (!finite(q)) {return fail("non-finite retimed position");}
    result.positions.push_back(std::move(q));
  }
  if (!validateCommandDerivatives(result.positions, dt, velocity, acceleration, jerk, result.error)) {
    result.positions.clear(); return result;
  }
  result.valid = true;
  return result;
}
}  // namespace motion_control_pkg::motion
#endif
