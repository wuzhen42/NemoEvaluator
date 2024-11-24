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

#include "AnimSequence.h"
#include <fstream>

namespace nemo {
std::unique_ptr<IChannel> make_single(const nlohmann::json &data, double fps) {
  if (data.count("animcurve")) {
    return std::make_unique<ChannelAnimCurve>(data["animcurve"], fps);
  } else if (data.count("constant")) {
    auto value = data["constant"];
    if (value.is_boolean())
      return std::make_unique<ChannelConstant>(value.get<bool>());
    else
      return std::make_unique<ChannelConstant>(value.get<double>());
  } else {
    throw std::runtime_error("unsupported channel type");
  }
}

Animation::Animation(std::string path) {
  std::ifstream fin(path);
  if (!fin.is_open())
    throw std::runtime_error("Could not load animation: " + path);
  nlohmann::json root = nlohmann::json::parse(fin);
  duration.first = root["framebegin"];
  duration.second = root["frameend"];
  this->fps = root["fps"];
  for (const auto &element : root["channels"].items()) {
    std::string name = element.key();
    std::string type = element.value()["type"];
    if (type == "double3") {
      std::unique_ptr<ChannelCompound> channel = std::make_unique<ChannelCompound>();
      channel->append(make_single(element.value()["components"][0], fps));
      channel->append(make_single(element.value()["components"][1], fps));
      channel->append(make_single(element.value()["components"][2], fps));
      channels.emplace_back(std::move(channel));
    } else {
      channels.emplace_back(make_single(element.value(), fps));
    }

    channels.back()->setName(name);
    channels.back()->setType(type);
  }
}
ChannelAnimCurve::ChannelAnimCurve(const nlohmann::json &data, double fps) {
  curve.weightedTangents = data.at("weightedTangents");
  curve.preInfinity = static_cast<InfinityType>(data.at("preInfinity"));
  curve.postInfinity = static_cast<InfinityType>(data.at("postInfinity"));

  const int numKeys = data.at("keyTime").size();
  for (int i = 0; i != numKeys; ++i) {
    AnimKeyFrame key;
    key.time = data.at("keyTime")[i] / fps;
    key.value = data.at("keyValue")[i];
    key.keyTanIn = {data.at("keyTanIn")[i][0], data.at("keyTanIn")[i][1]};
    key.keyTanInType = static_cast<TangentType>(data.at("keyTanInType")[i].get<int>());
    key.keyTanOut = {data.at("keyTanOut")[i][0], data.at("keyTanOut")[i][1]};
    key.keyTanOutType = static_cast<TangentType>(data.at("keyTanOutType")[i].get<int>());
    curve.keys.push_back(key);
  }
}

double ChannelAnimCurve::getReal(double frame) const { return curve.evaluate(frame); }

ChannelConstant::ChannelConstant(double value) : value(value) {}

double ChannelConstant::getReal(double frame) const { return value; }

void ChannelCompound::append(std::unique_ptr<IChannel> &&element) { components.emplace_back(std::move(element)); }

glm::dvec3 ChannelCompound::getVec3(double frame) const {
  return {components[0]->getReal(frame), components[1]->getReal(frame), components[2]->getReal(frame)};
}

} // namespace nemo
