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

#include <Alembic/Abc/All.h>
#include <Alembic/AbcCoreAbstract/All.h>
#include <Alembic/AbcCoreOgawa/All.h>
#include <Alembic/AbcGeom/All.h>
#include <argparse/argparse.hpp>
#include <boost/algorithm/string.hpp>
#include <fmt/format.h>
#include <future>
#include <nlohmann/json.hpp>
#include <vector>

#include "AnimSequence.h"
#include "Evaluate.h"

void execute(std::string pathConfig, std::string pathAnimation, bool profile) {
  try {
    nemo::Evaluator evaluator(pathConfig);
    evaluator.setAnimation(pathAnimation);
    std::string path_cache = pathAnimation;
    boost::algorithm::replace_last(path_cache, ".json", ".abc");
    auto [first, last] = evaluator.duration();

    using namespace Alembic;
    const AbcCoreAbstract::chrono_t timePerCycle = 1.0 / 24.0;
    std::vector<AbcCoreAbstract::chrono_t> tvec{first * timePerCycle};
    const AbcCoreAbstract::TimeSamplingType tSampTyp(1, timePerCycle);
    const AbcCoreAbstract::TimeSampling tSamp(tSampTyp, tvec);

    Abc::OArchive oarchive(AbcCoreOgawa::WriteArchive(), path_cache, Abc::ErrorHandler::kThrowPolicy);
    Abc::OObject top = oarchive.getTop();
    unsigned tsidx = top.getArchive().addTimeSampling(tSamp);

    std::vector<AbcGeom::OPolyMeshSchema> meshes;
    unsigned totalMS = 0;
    for (int frame = first; frame <= last; ++frame) {
      std::chrono::time_point tp_begin = std::chrono::high_resolution_clock::now();

      evaluator.updateInputsFromAnimation(frame);
      evaluator.evaluate();

      unsigned ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - tp_begin).count();
      if (profile) {
        if (frame != first)
          totalMS += ms;
        continue;
      }

      std::cout << fmt::format("frame{} taken {}ms", frame, ms) << std::endl;
      // set outputs
      for (unsigned mesh_id = 0; mesh_id != evaluator.meshes.size(); ++mesh_id) {
        unsigned plug_id = evaluator.meshes[mesh_id];

        std::vector<glm::vec3> points = evaluator.getPoints(plug_id);
        std::vector<Abc::V3f> positions(points.size());
        for (int i = 0; i != points.size(); ++i)
          positions[i] = {points[i].x, points[i].y, points[i].z};

        if (frame == first) {
          nemo::Evaluator::PlugInfo info = evaluator.plugAt(plug_id);
          std::string shapename = info.name.substr(0, info.name.find("__DOT__"));
          boost::algorithm::replace_all(shapename, ":", "__COLON__");
          std::string xformname = boost::algorithm::ends_with(shapename, "Shape") ? shapename.substr(0, shapename.rfind("Shape")) : shapename;
          AbcGeom::OXform xformObj{top, xformname, tsidx};
          AbcGeom::OPolyMesh meshObj(xformObj, shapename);

          meshes.emplace_back(meshObj.getSchema());
          AbcGeom::OPolyMeshSchema &mesh = meshes.back();
          mesh.setTimeSampling(tsidx);

          auto [counts, connection] = evaluator.getTopo(plug_id);
          AbcGeom::OPolyMeshSchema::Sample sample;
          std::vector<int> faceCounts{counts.begin(), counts.end()};
          std::vector<int> faceIndices{connection.begin(), connection.end()};
          sample.setFaceCounts(faceCounts);
          sample.setFaceIndices(faceIndices);
          sample.setPositions(positions);
          mesh.set(sample);
        } else {
          AbcGeom::OPolyMeshSchema::Sample sample;
          sample.setPositions(positions);
          meshes[mesh_id].set(sample);
        }
      }
    }
  } catch (const std::exception &err) {
    std::cerr << err.what() << std::endl;
  }
}

int main(int argc, char *argv[]) {
  std::string path_config, path_animation;
  argparse::ArgumentParser program("NemoEvaluator");
  program.add_argument("anim").help("animation sequence json");
  program.add_argument("config").help("nemo config json");
  program.add_argument("--instances").help("profiling instances").default_value(std::string{"0"});

  int N = 0;
  try {
    program.parse_args(argc, argv);
    path_animation = program.get("anim");
    path_config = program.get("config");
    N = std::stoi(program.get("--instances"));
  } catch (const std::runtime_error &err) {
    std::cerr << err.what() << std::endl;
    std::cerr << program;
    std::exit(1);
  }

  if (N > 0) {
    std::chrono::time_point tp_begin = std::chrono::high_resolution_clock::now();
    std::vector<std::thread> threads;
    for (int i = 0; i != N; ++i)
      threads.emplace_back(execute, path_config, path_animation, true);
    for (auto &thread : threads)
      thread.join();
    unsigned ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - tp_begin).count();
    std::cout << fmt::format("taken {}ms", ms) << std::endl;
  } else {
    execute(path_config, path_animation, false);
  }
}