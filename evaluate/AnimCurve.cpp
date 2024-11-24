/*
 * MIT License
 *
 * Copyright (c) 2024 wuzhen
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * 1. The above copyright notice and this permission notice shall be included in
 *    all copies or substantial portions of the Software.
 *
 * 2. THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *    SOFTWARE.
 */

#include "AnimCurve.h"
#include <algorithm>
#include <array>
#include <cassert>
#include <complex>
#include <glm/gtx/norm.hpp>

std::array<double, 4> IMPL__bezier_to_poly(std::array<double, 4> x) {
  double a = x[1] - x[0];
  double b = x[2] - x[1];
  double c = x[3] - x[2];
  double d = b - a;

  std::array<double, 4> result;
  result[3] = c - b - d;
  result[2] = d + d + d;
  result[1] = a + a + a;
  result[0] = x[0];
  return result;
}

double IMPL__solve_quadratic(std::array<double, 3> ce) {
  double delta = ce[1] * ce[1] - 4 * ce[0] * ce[2];
  assert(delta >= 0);
  double root1 = (-ce[1] + std::sqrt(delta)) / (2 * ce[0]);
  if (0 - 1E-3 <= root1 && root1 <= 1 + 1E-3)
    return root1;
  double root2 = (-ce[1] - std::sqrt(delta)) / (2 * ce[0]);
  assert(0 - 1E-3 <= root2 && root2 <= 1 + 1E-3);
  return root2;
}

double IMPL__solve_cubic(std::array<double, 3> ce) {
  double shift = (1.0 / 3) * ce[0];
  double p = ce[1] - shift * ce[0];
  double q = ce[0] * ((2.0 / 27.0) * ce[0] * ce[0] - (1.0 / 3.0) * ce[1]) + ce[2];

  double discriminant = (1.0 / 27.0) * p * p * p + (1.0 / 4.0) * q * q;
  if (discriminant >= 0) {
    double u = std::cbrt(-q * 0.5 + std::sqrt(discriminant));
    double v = (-1.0 / 3.0) * p / u;
    double root = u + v - shift;
    return root;
  }

  std::complex<double> u(q, 0), rt[3];
  u = std::pow(double{-0.5} * u - std::sqrt(double{0.25} * u * u + p * p * p / double{27.0}), 1.0 / 3.0);
  rt[0] = u - p / (double{3.0} * u) - shift;
  std::complex<double> w(-0.5, std::sqrt(3.0) / 2);
  rt[1] = u * w - p / (double{3.0} * u * w) - shift;
  rt[2] = u / w - p * w / (double{3.0} * u) - shift;

  if (0 - 1E-3 <= rt[0].real() && rt[0].real() <= 1 + 1E-3) {
    return rt[0].real();
  } else if (0 - 1E-3 <= rt[1].real() && rt[1].real() <= 1 + 1E-3) {
    return rt[1].real();
  } else {
    return rt[2].real();
  }
}

double IMPL__bezier_evaluate(glm::dvec2 p0, glm::dvec2 p1, glm::dvec2 p2, glm::dvec2 p3, double input) {
  assert(p0.x <= input && input <= p3.x);
  double rangeX = p3.x - p0.x;
  if (rangeX <= 1E-5)
    return 0.0;

  double nX1 = (p1.x - p0.x) / rangeX;
  double nX2 = (p2.x - p0.x) / rangeX;

  double t;
  double s = (input - p0.x) / rangeX;

  constexpr double oneThird = 1.0 / 3.0;
  constexpr double twoThirds = 2.0 / 3.0;
  if (std::abs(nX1 - oneThird) < 1E-5 && std::abs(nX2 - twoThirds) < 1E-5) {
    t = s;
  } else {
    auto polyX = IMPL__bezier_to_poly({0.0, nX1, nX2, 1.0});
    if (std::abs(polyX[3]) > 1E-5)
      t = IMPL__solve_cubic({polyX[2] / polyX[3], polyX[1] / polyX[3], (polyX[0] - s) / polyX[3]});
    else
      t = IMPL__solve_quadratic({polyX[2], polyX[1], polyX[0] - s});
  }

  auto polyY = IMPL__bezier_to_poly({p0.y, p1.y, p2.y, p3.y});
  return t * (t * (t * polyY[3] + polyY[2]) + polyY[1]) + polyY[0];
}

double IMPL__hermit_evaluate(glm::dvec2 p0, glm::dvec2 p1, glm::dvec2 p2, glm::dvec2 p3, double input) {
  assert(p0.x <= input && input <= p3.x);
  double dx = p3.x - p0.x;
  double dy = p3.y - p0.y;

  double tan_x = p1.x - p0.x;
  double m1 = 0.0;
  if (tan_x != 0.0)
    m1 = (p1.y - p0.y) / tan_x;

  tan_x = p3.x - p2.x;
  double m2 = 0.0;
  if (tan_x != 0.0)
    m2 = (p3.y - p2.y) / tan_x;

  double length = 1.0 / (dx * dx);
  double double1 = dx * m1;
  double double2 = dx * m2;
  double fCoeff[4];
  fCoeff[0] = (double1 + double2 - dy - dy) * length / dx;
  fCoeff[1] = (dy + dy + dy - double1 - double1 - double2) * length;
  fCoeff[2] = m1;
  fCoeff[3] = p0.y;

  double t = input - p0.x;
  return t * (t * (t * fCoeff[0] + fCoeff[1]) + fCoeff[2]) + fCoeff[3];
}

double AnimCurve::evaluate(double input) const {
  double offset = 0;

  AnimKeyFrame first = keys.front();
  AnimKeyFrame last = keys.back();
  if (input <= first.time) {
    switch (preInfinity) {
    case InfinityType::kConstant:
      return first.value;
    case InfinityType::kLinear: {
      double ratio = first.keyTanIn.x == 0 ? 0 : first.keyTanIn.y / first.keyTanIn.x;
      return first.value - ratio * (first.time - input);
    }
    case InfinityType::kCycle:
      if (last.time - first.time > 1E-5) {
        while (input < first.time) {
          input += (last.time - first.time);
        }
      }
      break;
    case InfinityType::kCycleOffset:
      if (last.time - first.time > 1E-5)
        return first.value;
      while (input < first.time) {
        input += (last.time - first.time);
        offset -= (last.value - first.value);
      }
      break;
    case InfinityType::kOscillate: {
      if (last.time - first.time > 1E-5)
        return first.value;
      bool flip = false;
      while (input < first.time) {
        input += (last.time - first.time);
        flip = !flip;
      }
      if (flip)
        input = last.time - (input - first.time);
    } break;
    default:
      return 0;
    }
  }

  if (input >= last.time) {
    switch (postInfinity) {
    case InfinityType::kConstant:
      return last.value;
    case InfinityType::kLinear: {
      double ratio = last.keyTanOut.x == 0 ? 0 : last.keyTanOut.y / last.keyTanOut.x;
      return last.value + ratio * (input - last.time);
    }
    case InfinityType::kCycle:
      if (last.time - first.time > 1E-5) {
        while (input >= last.time) {
          input -= (last.time - first.time);
        }
      }
      break;
    case InfinityType::kCycleOffset:
      if (last.time - first.time < 1E-5)
        return last.value;
      while (input >= last.time) {
        input -= (last.time - first.time);
        offset += (last.value - first.value);
      }
      break;
    case InfinityType::kOscillate: {
      if (last.time - first.time < 1E-5)
        return last.value;
      bool flip = false;
      while (input >= first.time) {
        input -= (last.time - first.time);
        flip = !flip;
      }
      if (flip)
        input = first.time + (last.time - input);
    } break;
    default:
      return 0;
    }
  }

  auto idx_end = std::max<unsigned>(1, std::distance(keys.begin(), std::upper_bound(keys.begin(), keys.end(), input,
                                                                                    [](double input, const AnimKeyFrame &key) { return input < key.time; })));
  const unsigned idx_begin = idx_end - 1;
  const AnimKeyFrame key0 = keys[idx_begin];
  const AnimKeyFrame key1 = keys[idx_end];

  if (TangentType::kStep == key0.keyTanOutType)
    return key0.value + offset;
  if (TangentType::kStepNext == key1.keyTanInType)
    return key1.value + offset;

  constexpr double oneThird = 1.0 / 3.0;
  glm::dvec2 tangentIn = key0.keyTanOut;
  glm::dvec2 tangentOut = key1.keyTanIn;

  if (std::abs(key0.value - key1.value) < 1E-5 && (std::abs(glm::normalize(tangentIn).y) < 1E-5) && (std::abs(glm::normalize(tangentOut).y) < 1E-5))
    return key0.value + offset;

  bool linear = TangentType::kLinear == key0.keyTanOutType && TangentType::kLinear == key1.keyTanInType;

  if (linear) {
    const double alpha = (input - key0.time) / (key1.time - key0.time);
    return (1 - alpha) * key0.value + alpha * key1.value + offset;
  }

  glm::dvec2 P0{key0.time, key0.value};
  glm::dvec2 P1 = P0 + tangentIn * oneThird;
  glm::dvec2 P3{key1.time, key1.value};
  glm::dvec2 P2 = P3 - tangentOut * oneThird;

  if (weightedTangents)
    return IMPL__bezier_evaluate(P0, P1, P2, P3, input) + offset;
  else
    return IMPL__hermit_evaluate(P0, P1, P2, P3, input) + offset;
}

template <typename T> int signNoZero(T val) { return (T(0) < val) ? -1 : 1; }

glm::dvec2 autoTangent(bool calculateInTangent, AnimKeyFrame key, AnimKeyFrame *prevKey, AnimKeyFrame *nextKey, int weighted) {
  glm::dvec2 tan;
  if (prevKey == nullptr || nextKey == nullptr) {
    // calculate flat tangent
    double tanInx = 0.0, tanIny = 0.0;
    double tanOutx = 0.0, tanOuty = 0.0;
    double x = key.time;

    if (prevKey != nullptr)
      tanInx = x - prevKey->time;

    if (nextKey != nullptr)
      tanOutx = nextKey->time - x;

    if (prevKey == nullptr)
      tanInx = tanOutx;

    if (nextKey == nullptr)
      tanOutx = tanInx;

    if (calculateInTangent) {
      tan = {tanInx, tanIny};
    } else {
      tan = {tanOutx, tanOuty};
    }
  } else {
    double x = key.time;
    double y = key.value;
    double px = prevKey->time;
    double nx = nextKey->time;
    double py = prevKey->value;
    double ny = nextKey->value;

    if (calculateInTangent)
      tan.x = x - px;
    else
      tan.x = nx - x;

    double targetSlope = 0.0, prevSlope3 = 0.0, nextSlope3 = 0.0;

    // Target slope is the default spline slope.
    // prevSlope3 and nextSlope3 are respectively the slopes to the left and right keys multiplied by 3.
    // Target slope needs to be adjusted to fit between these 2 last slope values to ensure that the
    // control points are not outside of the Y range defined by the prev and next keys

    targetSlope = (ny - py) / (nx - px);
    prevSlope3 = 3.0 * (y - py) / (x - px);
    nextSlope3 = 3.0 * (ny - y) / (nx - x);

    if (signNoZero(prevSlope3) != signNoZero(nextSlope3) || signNoZero(targetSlope) != signNoZero(nextSlope3)) {
      targetSlope = 0.0;
    } else if (nextSlope3 >= 0) {
      targetSlope = std::min(std::min(targetSlope, nextSlope3), prevSlope3);
    } else {
      targetSlope = std::max(std::max(targetSlope, nextSlope3), prevSlope3);
    }
    tan.y = targetSlope * tan.x;
  }

  if (tan.x < 0)
    tan.x = 0;

  if (!weighted) {
    double length = sqrt(tan.x * tan.x + tan.y * tan.y);
    if (length > 0) {
      tan /= length;
    }
  }
  return tan;
}

void AnimCurve::updateAutoTangent() {
  if (this->size() <= 2)
    return;

  for (int i = 0; i != this->size(); ++i) {
    const bool hasPrev = (i > 0);
    const bool hasNext = (i + 1 < this->size());
    if (keys[i].keyTanInType == TangentType::kAuto)
      keys[i].keyTanIn = autoTangent(true, keys[i], hasPrev ? &keys[i - 1] : nullptr, hasNext ? &keys[i + 1] : nullptr, this->weightedTangents);
    if (keys[i].keyTanOutType == TangentType::kAuto)
      keys[i].keyTanOut = autoTangent(true, keys[i], hasPrev ? &keys[i - 1] : nullptr, hasNext ? &keys[i + 1] : nullptr, this->weightedTangents);
  }
}
