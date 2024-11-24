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

#include <OP/OP_OperatorTable.h>
#include <UT/UT_DSOVersion.h>

#include "SOP_Nemo.h"
#include "SOP_NemoPlay.h"

void newSopOperator(OP_OperatorTable *table) {
  table->addOperator(new OP_Operator("nemo", "Nemo", SOP_Nemo::myConstructor, SOP_Nemo::myTemplateList, 0, 0, nullptr, OP_FLAG_GENERATOR));
  table->addOperator(new OP_Operator("nemoplay", "NemoPlay", SOP_NemoPlay::myConstructor, SOP_NemoPlay::myTemplateList, 0, 0, nullptr, OP_FLAG_GENERATOR));
}
