/*
 * MIT License
 *
 * Copyright (c) 2023 wuzhen
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

#include "SOP_Nemo.h"
#include <GA/GA_Names.h>
#include <GEO/GEO_PrimPoly.h>
#include <GU/GU_Detail.h>
#include <OP/OP_AutoLockInputs.h>
#include <OP/OP_Director.h>
#include <OP/OP_NodeInfoParms.h>
#include <UT/UT_InfoTree.h>

#include <boost/algorithm/string.hpp>
#include <glm/glm.hpp>

#include "Evaluate.h"

static PRM_Name sopNemoConfigName("config", "Nemo Config");
static PRM_Name sopNemoAnimName("anim", "Animation");
static PRM_Name sopNemoFrameName("frame", "Frame");

static PRM_Default prm_frameDefault(1, "$FF");
PRM_Template SOP_Nemo::myTemplateList[] = {PRM_Template(PRM_FILE_E, 1, &sopNemoConfigName, 0), PRM_Template(PRM_FILE_E, 1, &sopNemoAnimName, 0),
                                           PRM_Template(PRM_FLT, 1, &sopNemoFrameName, &prm_frameDefault), PRM_Template()};

OP_Node *SOP_Nemo::myConstructor(OP_Network *net, const char *name, OP_Operator *op) { return new SOP_Nemo(net, name, op); }

SOP_Nemo::SOP_Nemo(OP_Network *net, const char *name, OP_Operator *op) : SOP_Node(net, name, op) {}

OP_ERROR SOP_Nemo::cookMySop(OP_Context &context) {
  OP_AutoLockInputs inputs(this);
  if (inputs.lock(context) >= UT_ERROR_ABORT)
    return error();

  bool init_geom = mesh_points_offset.empty();

  fpreal now = context.getTime();
  UT_StringHolder config, anim;
  evalString(config, "config", 0, now);
  evalString(anim, "anim", 0, now);
  fpreal frame = evalFloat("frame", 0, now);
  if (config != cachedPathConfig || anim != cachedPathAnim) {
    cachedPathConfig = config;
    cachedPathAnim = anim;
    if (cachedPathConfig.length() > 0 && cachedPathAnim.length() > 0) {
      evaluator = std::make_unique<nemo::Evaluator>(cachedPathConfig.toStdString(), cachedPathAnim.toStdString());
      init_geom = true;
    }
  }
  if (!evaluator)
    return error();
  evaluator->evaluate(frame);

  // check visibility
  if (cachedMeshVisibility.empty())
    cachedMeshVisibility.resize(evaluator->meshes.size(), false);
  for (unsigned mesh_id = 0; mesh_id != evaluator->meshes.size(); ++mesh_id) {
    unsigned plug_id = evaluator->meshes[mesh_id];
    bool visible = evaluator->isVisible(plug_id);
    if (visible != cachedMeshVisibility[mesh_id]) {
      init_geom = true;
      cachedMeshVisibility[mesh_id] = visible;
    }
  }

  if (init_geom) {
    mesh_points_offset.clear();
    gdp->clear();
  }

  std::vector<glm::vec3> all_points;
  if (!init_geom)
    all_points.reserve(mesh_points_offset.back());

  std::map<std::string, unsigned> meshes;
  for (unsigned mesh_id = 0; mesh_id != evaluator->meshes.size(); ++mesh_id) {
    unsigned plug_id = evaluator->meshes[mesh_id];
    std::string path = evaluator->LUT_path.at(plug_id);
    boost::algorithm::replace_all(path, "|", "/");
    meshes[path] = plug_id;
  }

  for (auto [_, plug_id] : meshes) {
    if (!evaluator->isVisible(plug_id))
      continue;
    std::vector<glm::vec3> points = evaluator->getPoints(plug_id);
    all_points.insert(all_points.end(), points.begin(), points.end());
  }

  if (init_geom)
    pointsStartOffset = gdp->appendPointBlock(all_points.size());

  for (unsigned i = 0; i != all_points.size(); ++i)
    gdp->setPos3(pointsStartOffset + i, all_points[i].x, all_points[i].y, all_points[i].z);

  if (init_geom) {
    GA_RWHandleS SHandle = gdp->addStringTuple(GA_ATTRIB_PRIMITIVE, "path", 1);
    GA_RWHandleV2 uvAttr = gdp->addFloatTuple(GA_ATTRIB_VERTEX, GA_Names::uv, 2);
    uvAttr->setTypeInfo(GA_TYPE_TEXTURE_COORD);

    unsigned offset = 0;
    unsigned totalFaceVertex = 0;
    for (auto [path, plug_id] : meshes) {
      if (!evaluator->isVisible(plug_id))
        continue;
      auto [counts, connection] = evaluator->getTopo(plug_id);
      std::vector<float> uValues, vValues;
      std::vector<unsigned> uvIds;
      std::tie(uValues, vValues, uvIds) = evaluator->getDefaultUV(plug_id);
      // assign (0,0) for vertex no uv attached
      if (uvIds.size() < connection.size()) {
        for (int t = uvIds.size(); t != connection.size(); ++t) {
          uvIds.push_back(uValues.size());
        }
        uValues.push_back(0.f);
        vValues.push_back(0.f);
      }

      unsigned p = 0;
      for (unsigned numFaceVtx : counts) {
        GEO_PrimPoly *poly = GEO_PrimPoly::build(gdp, numFaceVtx, GU_POLY_CLOSED, false);
        for (int i = 0; i != numFaceVtx; ++i) {
          int vtxOffset = p + (numFaceVtx - 1 - i);
          poly->setVertexPoint(i, offset + connection[vtxOffset]);
          unsigned idxUV = uvIds[vtxOffset];
          uvAttr.set(totalFaceVertex + p + i, UT_Vector2(uValues[idxUV], vValues[idxUV]));
        }
        p += numFaceVtx;
        SHandle.set(poly->getMapOffset(), path);
      }
      totalFaceVertex += p;
      mesh_points_offset.push_back(offset);
      offset += evaluator->getPoints(plug_id).size();
    }
    mesh_points_offset.push_back(offset);
  }

  return error();
}

void SOP_Nemo::fillInfoTreeNodeSpecific(UT_InfoTree &tree, const OP_NodeInfoTreeParms &parms) {
  SOP_Node::fillInfoTreeNodeSpecific(tree, parms);
  UT_InfoTree *branch = tree.addChildMap("NemoPlay SOP Info");
  branch->addProperties("Start Frame", evaluator->duration().first);
  branch->addProperties("End Frame", evaluator->duration().second);
}
