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

#pragma once
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <vector>

enum class TangentType { kFixed = 1, kLinear = 2, kFlat = 3, kStep = 5, kSpline = 9, kStepNext = 17, kAuto = 18 };

enum class InfinityType { kConstant = 0, kLinear = 1, kCycle = 3, kCycleOffset = 4, kOscillate = 5 };

struct AnimKeyFrame {
  double time;
  double value;
  glm::dvec2 keyTanIn;
  TangentType keyTanInType;
  glm::dvec2 keyTanOut;
  TangentType keyTanOutType;
};

struct AnimCurve {
  std::vector<AnimKeyFrame> keys;
  InfinityType preInfinity;
  InfinityType postInfinity;
  bool weightedTangents;
  bool isRotate = false;

  double evaluate(double time) const;

  void updateAutoTangent();

  unsigned size() const { return keys.size(); }
};