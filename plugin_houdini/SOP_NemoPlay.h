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
#include "AnimSequence.h"
#include <SOP/SOP_Node.h>

class SOP_NemoPlay : public SOP_Node {
  std::unique_ptr<nemo::Evaluator> evaluator;
  std::unique_ptr<nemo::Animation> animation;
  std::vector<unsigned> mesh_points_offset;
  GA_Offset pointsStartOffset;

  UT_StringHolder cachedPathConfig, cachedPathAnim;
  std::vector<bool> cachedMeshVisibility;

public:
  static OP_Node *myConstructor(OP_Network *, const char *, OP_Operator *);

  static PRM_Template myTemplateList[];

private:
  SOP_NemoPlay(OP_Network *net, const char *name, OP_Operator *op);

  OP_ERROR cookMySop(OP_Context &context) override;

  void fillInfoTreeNodeSpecific(UT_InfoTree &tree, const OP_NodeInfoTreeParms &parms) override;
};