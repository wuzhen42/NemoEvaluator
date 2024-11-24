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
#include <GA/GA_ElementGroup.h>
#include <GA/GA_Names.h>
#include <GEO/GEO_PrimPoly.h>
#include <GU/GU_Detail.h>
#include <OP/OP_AutoLockInputs.h>
#include <OP/OP_Director.h>
#include <OP/OP_NodeInfoParms.h>
#include <UT/UT_InfoTree.h>

#include <boost/algorithm/string.hpp>
#include <glm/ext/vector_relational.hpp>
#include <glm/glm.hpp>
#include <iostream>

#include "../nemo/NameUtils.hpp"
#include "Evaluate.h"

static PRM_Name sopNemoConfigName("config", "Config JSON");
static PRM_Name nameVector[] = {PRM_Name("vector", "Vector"), PRM_Name("name_vector#", "Name Vector #"), PRM_Name("vector#", " Vector #"), PRM_Name(0)};
static PRM_Template paramVector[] = {PRM_Template(PRM_STRING, 1, &nameVector[1]), PRM_Template(PRM_XYZ, 3, &nameVector[2]), PRM_Template()};
static PRM_Name nameFloat[] = {PRM_Name("float", "Float"), PRM_Name("name_float#", "Name Float #"), PRM_Name("float#", "Float #"), PRM_Name(0)};
static PRM_Template paramFloat[] = {PRM_Template(PRM_STRING, 1, &nameFloat[1]), PRM_Template(PRM_FLT, 1, &nameFloat[2]), PRM_Template()};
static PRM_Name nameInt[] = {PRM_Name("int", "Int"), PRM_Name("name_int#", "Name Int #"), PRM_Name("int#", "Int #"), PRM_Name(0)};
static PRM_Template paramInt[] = {PRM_Template(PRM_STRING, 1, &nameInt[1]), PRM_Template(PRM_FLT, 1, &nameInt[2]), PRM_Template()};
PRM_Template SOP_Nemo::myTemplateList[] = {PRM_Template(PRM_FILE_E, 1, &sopNemoConfigName, 0), PRM_Template(PRM_MULTITYPE_LIST, paramVector, 3, &nameVector[0]),
                                           PRM_Template(PRM_MULTITYPE_LIST, paramFloat, 1, &nameFloat[0]),
                                           PRM_Template(PRM_MULTITYPE_LIST, paramInt, 1, &nameInt[0]), PRM_Template()};

OP_Node *SOP_Nemo::myConstructor(OP_Network *net, const char *name, OP_Operator *op) { return new SOP_Nemo(net, name, op); }

SOP_Nemo::SOP_Nemo(OP_Network *net, const char *name, OP_Operator *op) : SOP_Node(net, name, op) {}

UT_Matrix4D readMatrix(const glm::dmat4 &mat) {
  UT_Matrix4D result;
  for (int i = 0; i != 4; ++i)
    for (int j = 0; j != 4; ++j)
      result[i][j] = mat[i][j];
  return result;
}

OP_ERROR SOP_Nemo::cookMySop(OP_Context &context) {
  OP_AutoLockInputs inputs(this);
  if (inputs.lock(context) >= UT_ERROR_ABORT)
    return error();

  fpreal now = initEvaluator(context);
  if (!evaluator)
    return error();

  updateInputs(now);
  evaluator->evaluate();

  if (!initGeom && mesh_points_offset.empty())
    initGeom = true;

  // output mesh
  // check visibility
  if (cachedMeshVisibility.empty())
    cachedMeshVisibility.resize(evaluator->meshes.size(), false);
  for (unsigned mesh_id = 0; mesh_id != evaluator->meshes.size(); ++mesh_id) {
    unsigned plug_id = evaluator->meshes[mesh_id];
    bool visible = evaluator->isVisible(plug_id);
    if (visible != cachedMeshVisibility[mesh_id]) {
      initGeom = true;
      cachedMeshVisibility[mesh_id] = visible;
    }
  }

  if (initGeom) {
    mesh_points_offset.clear();
    gdp->clear();
  }

  std::vector<glm::vec3> all_points;
  if (!initGeom)
    all_points.reserve(mesh_points_offset.back());

  std::map<std::string, unsigned> meshes;
  for (unsigned mesh_id = 0; mesh_id != evaluator->meshes.size(); ++mesh_id) {
    unsigned plug_id = evaluator->meshes[mesh_id];
    std::string path = evaluator->LUT_path.at(plug_id);
    boost::algorithm::replace_all(path, "|", "/");
    meshes[path] = plug_id;
  }

  for (auto [path, plug_id] : meshes) {
    if (!evaluator->isVisible(plug_id))
      continue;
    std::vector<glm::vec3> points = evaluator->getPoints(plug_id);
    all_points.insert(all_points.end(), points.begin(), points.end());
  }

  if (initGeom)
    pointsStartOffset = gdp->appendPointBlock(all_points.size());

  for (unsigned i = 0; i != all_points.size(); ++i)
    gdp->setPos3(pointsStartOffset + i, all_points[i].x, all_points[i].y, all_points[i].z);

  if (initGeom) {
    GA_RWHandleS SHandle = gdp->addStringTuple(GA_ATTRIB_PRIMITIVE, "path", 1);
    GA_RWHandleV2 uvAttr = gdp->addFloatTuple(GA_ATTRIB_VERTEX, GA_Names::uv, 2);
    uvAttr->setTypeInfo(GA_TYPE_TEXTURE_COORD);

    std::map<std::string, GA_PrimitiveGroup *> groups;
    for (const auto &faceset : evaluator->facesets) {
      groups[faceset.name] = gdp->newPrimitiveGroup(faceset.name);
    }

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

      std::vector<std::pair<GA_PrimitiveGroup *, std::set<unsigned>>> primitiveGroup;
      for (const auto &faceset : evaluator->facesets) {
        if (faceset.members.count(plug_id)) {
          const std::vector<unsigned> &elements = faceset.members.at(plug_id);
          primitiveGroup.emplace_back(groups.at(faceset.name), std::set<unsigned>{elements.begin(), elements.end()});
        }
      }

      unsigned p = 0;
      for (int idxFace = 0; idxFace != counts.size(); ++idxFace) {
        unsigned numFaceVtx = counts[idxFace];
        GEO_PrimPoly *poly = GEO_PrimPoly::build(gdp, numFaceVtx, GU_POLY_CLOSED, false);
        for (int i = 0; i != numFaceVtx; ++i) {
          int vtxOffset = p + (numFaceVtx - 1 - i);
          poly->setVertexPoint(i, offset + connection[vtxOffset]);
          unsigned idxUV = uvIds[vtxOffset];
          uvAttr.set(totalFaceVertex + p + i, UT_Vector2(uValues[idxUV], vValues[idxUV]));
        }
        p += numFaceVtx;
        SHandle.set(poly->getMapOffset(), path);
        for (const auto &[group, members] : primitiveGroup) {
          if (members.empty()) {
            group->addOffset(poly->getMapOffset());
            break;
          }
          if (members.count(idxFace))
            group->addOffset(poly->getMapOffset());
        }
      }
      totalFaceVertex += p;
      mesh_points_offset.push_back(offset);
      offset += evaluator->getPoints(plug_id).size();
    }
    mesh_points_offset.push_back(offset);
    initGeom = false;
  }

  writeDataOutputs(gdp, all_points.size());
  return error();
}

std::vector<unsigned> SOP_Nemo::loadPlugs(PRM_Name names[], fpreal now) {
  int numInstances = evalInt(names[0], 0, now);
  std::vector<unsigned> plugsId(numInstances);
  int startIndex = getParm(names[0]).getMultiStartOffset();
  for (int i = 0; i != numInstances; ++i) {
    int instanceIndex = startIndex + i;
    UT_StringHolder name;
    evalStringInst(names[1], &instanceIndex, name, 0, now);
    unsigned idxPlug = evaluator->findPlugByName(var_name(name.toStdString()));
    plugsId[i] = idxPlug;
  }
  return plugsId;
}

void SOP_Nemo::writeDataOutputs(GU_Detail *dst, unsigned offset) {
  auto dataRange = evaluator->outputRange();
  unsigned dataSize = dataRange.second - dataRange.first;
  if (dst->getNumPoints() != offset + dataSize) {
    dst->appendPointBlock(offset + dataSize - dst->getNumPoints());
  }

  nemo::Runtime &runtime = evaluator->runtime;
  GA_RWHandleS nameHandle = dst->findStringTuple(GA_ATTRIB_POINT, "name");
  if (nameHandle.isInvalid())
    nameHandle = dst->addStringTuple(GA_ATTRIB_POINT, "name", 1);
  GA_RWHandleS datatypeHandle = dst->findStringTuple(GA_ATTRIB_POINT, "datatype");
  if (datatypeHandle.isInvalid())
    datatypeHandle = dst->addStringTuple(GA_ATTRIB_POINT, "datatype", 1);
  GA_RWHandleV3 rotateHandle = dst->findFloatTuple(GA_ATTRIB_POINT, "R");
  if (rotateHandle.isInvalid())
    rotateHandle = dst->addFloatTuple(GA_ATTRIB_POINT, "R", 3);
  GA_RWHandleV3 scaleHandle = dst->findFloatTuple(GA_ATTRIB_POINT, "S");
  if (scaleHandle.isInvalid())
    scaleHandle = dst->addFloatTuple(GA_ATTRIB_POINT, "S", 3);

  for (auto [idx, end] = dataRange; idx != end; ++idx) {
    const nemo::Evaluator::PlugInfo &info = evaluator->plugAt(idx);
    if (info.dataTypeStr == "Mesh" || info.dataTypeStr == "CuShape") {
      offset -= 1;
      continue;
    }

    const unsigned slot = offset + idx - dataRange.first;
    nameHandle.set(slot, info.name);
    datatypeHandle.set(slot, info.dataTypeStr);
    if ("Mat4" == info.dataTypeStr) {
      glm::dmat4 matrix;
      if (runtime.singlePrecision)
        matrix = runtime.data.getMat4(info.dataIndex);
      else
        matrix = runtime.data.getDMat4(info.dataIndex);
      UT_Matrix4D mat = readMatrix(matrix);
      UT_Vector3D translate, rotate;
      UT_Matrix3D stretch;
      mat.decompose(UT_XformOrder(), translate, rotate, stretch);
      rotate.radToDeg();
      dst->setPos3(slot, UT_Vector3D{matrix[3][0], matrix[3][1], matrix[3][2]});
      rotateHandle.set(slot, rotate);
      scaleHandle.set(slot, UT_Vector3D{stretch[0][0], stretch[1][1], stretch[2][2]});
    } else if ("Int" == info.dataTypeStr) {
      int value = runtime.data.getInt(info.dataIndex);
      dst->setPos3(slot, UT_Vector3D(value, value, value));
    } else if ("Bool" == info.dataTypeStr) {
      bool value = runtime.data.getBool(info.dataIndex);
      dst->setPos3(slot, UT_Vector3D(value, value, value));
    } else if ("Decimal" == info.dataTypeStr || "Angle" == info.dataTypeStr) {
      double value;
      if (runtime.singlePrecision)
        value = runtime.data.getFloat(info.dataIndex);
      else
        value = runtime.data.getDouble(info.dataIndex);
      if ("Angle" == info.dataTypeStr)
        value = glm::degrees(value);
      dst->setPos3(slot, UT_Vector3D{value, value, value});
    } else if ("Vec3" == info.dataTypeStr || "Euler" == info.dataTypeStr) {
      glm::dvec3 value;
      if (runtime.singlePrecision)
        value = runtime.data.getVec3(info.dataIndex);
      else
        value = runtime.data.getDVec3(info.dataIndex);
      if ("Euler" == info.dataTypeStr)
        value = glm::degrees(value);
      dst->setPos3(slot, UT_Vector3D{value.x, value.y, value.z});
    } else {
      std::cerr << "unknown output: " + info.name + "(" + info.dataTypeStr + ")" << std::endl;
    }
  }
}

void SOP_Nemo::updateInputs(fpreal now) {
  nemo::Runtime &runtime = evaluator->runtime;

  int startIndexVec = getParm(nameVector[0]).getMultiStartOffset();
  for (int i = 0, instId = startIndexVec; i != vectorInputPlugs.size(); ++i, ++instId) {
    float x = evalFloatInst(nameVector[2], &instId, 0, now);
    float y = evalFloatInst(nameVector[2], &instId, 1, now);
    float z = evalFloatInst(nameVector[2], &instId, 2, now);
    const nemo::Evaluator::PlugInfo &info = evaluator->plugAt(vectorInputPlugs[i]);

    if (runtime.singlePrecision) {
      if ("Euler" == info.dataTypeStr)
        runtime.data.setVec3(info.dataIndex, glm::radians(glm::vec3{x, y, z}));
      else
        runtime.data.setVec3(info.dataIndex, {x, y, z});
    } else {
      if ("Euler" == info.dataTypeStr)
        runtime.data.setDVec3(info.dataIndex, glm::radians(glm::dvec3{x, y, z}));
      else
        runtime.data.setDVec3(info.dataIndex, {x, y, z});
    }
  }

  int startIndexInt = getParm(nameInt[0]).getMultiStartOffset();
  for (int i = 0, instId = startIndexInt; i != intInputPlugs.size(); ++i, ++instId) {
    int value = evalIntInst(nameInt[2], &instId, 0, now);
    const nemo::Evaluator::PlugInfo &info = evaluator->plugAt(intInputPlugs[i]);
    if ("Bool" == info.dataTypeStr)
      runtime.data.setBool(info.dataIndex, value);
    else
      runtime.data.setInt(info.dataIndex, value);
  }

  int startIndexFloat = getParm(nameFloat[0]).getMultiStartOffset();
  for (int i = 0, instId = startIndexFloat; i != floatInputPlugs.size(); ++i, ++instId) {
    float value = evalFloatInst(nameFloat[2], &instId, 0, now);
    const nemo::Evaluator::PlugInfo &info = evaluator->plugAt(floatInputPlugs[i]);
    if (runtime.singlePrecision) {
      if ("Angle" == info.dataTypeStr)
        runtime.data.setFloat(info.dataIndex, glm::radians(value));
      else
        runtime.data.setFloat(info.dataIndex, value);
    } else {
      if ("Angle" == info.dataTypeStr)
        runtime.data.setDouble(info.dataIndex, glm::radians(value));
      else
        runtime.data.setDouble(info.dataIndex, value);
    }
  }
}

fpreal SOP_Nemo::initEvaluator(OP_Context &context) {
  fpreal now = context.getTime();
  UT_StringHolder configPath;
  evalString(configPath, "config", 0, now);

  if (configPath != cachedConfigPath) {
    cachedConfigPath = configPath;
    if (cachedConfigPath.length() == 0)
      return now;
    evaluator = std::make_unique<nemo::Evaluator>(cachedConfigPath.toStdString());
    initGeom = true;

    vectorInputPlugs = loadPlugs(nameVector, now);
    intInputPlugs = loadPlugs(nameInt, now);
    floatInputPlugs = loadPlugs(nameFloat, now);
  }
  return now;
}
