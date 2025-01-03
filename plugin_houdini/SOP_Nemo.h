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

#pragma once
#include "Evaluate.h"
#include <SOP/SOP_Node.h>

class SOP_Nemo : public SOP_Node {
  std::unique_ptr<nemo::Evaluator> evaluator;
  UT_StringHolder cachedConfigPath;

  std::vector<unsigned> vectorInputPlugs;
  std::vector<unsigned> intInputPlugs;
  std::vector<unsigned> floatInputPlugs;

  std::vector<unsigned> mesh_points_offset;
  GA_Offset pointsStartOffset;
  std::vector<bool> cachedMeshVisibility;
  bool initGeom = false;

public:
  static OP_Node *myConstructor(OP_Network *, const char *, OP_Operator *);

  static PRM_Template myTemplateList[];

private:
  SOP_Nemo(OP_Network *net, const char *name, OP_Operator *op);

  OP_ERROR cookMySop(OP_Context &context) override;

private:
  std::vector<unsigned> loadPlugs(PRM_Name names[], fpreal now);

  void writeDataOutputs(GU_Detail *dst, unsigned offset);

  void updateInputs(fpreal now);

  fpreal initEvaluator(OP_Context &context);
};