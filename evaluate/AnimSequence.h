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
#include <array>
#include <map>
#include <string>
#include <variant>
#include <vector>

#include "AnimCurve.h"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <nlohmann/json.hpp>

namespace nemo {
class IChannel {
  std::string name;
  std::string type;

public:
  const std::string &getName() { return name; }

  void setName(const std::string &name) { this->name = name; }

  const std::string &getType() const { return type; }

  void setType(const std::string &type) { this->type = type; }

  virtual double getReal(double time) const { return 0; }

  virtual glm::dvec3 getVec3(double time) const { return {0, 0, 0}; }

  virtual glm::dmat4 getMat4(double time) const { return glm::identity<glm::dmat4>(); }
};

struct Animation {
  std::vector<std::unique_ptr<IChannel>> channels;
  std::pair<int, int> duration;
  double fps;

  explicit Animation(std::string path);

  double getReal(unsigned id, float frame) { return channels[id]->getReal(frame / fps); }

  glm::dvec3 getVec3(unsigned id, float frame) { return channels[id]->getVec3(frame / fps); }

  glm::dmat4 getMat4(unsigned id, float frame) { return channels[id]->getMat4(frame / fps); }

private:
  static bool isIntegral(float x) { return std::abs(x - std::roundf(x)) < 1E-5F; }
};

class ChannelAnimCurve : public IChannel {
  AnimCurve curve;

public:
  explicit ChannelAnimCurve(const nlohmann::json &data, double fps);

  double getReal(double time) const override;
};

class ChannelConstant : public IChannel {
  double value;

public:
  explicit ChannelConstant(double value);

  double getReal(double time) const override;
};

class ChannelCompound : public IChannel {
  std::vector<std::unique_ptr<IChannel>> components;

public:
  void append(std::unique_ptr<IChannel> &&);

  glm::dvec3 getVec3(double time) const override;
};

} // namespace nemo