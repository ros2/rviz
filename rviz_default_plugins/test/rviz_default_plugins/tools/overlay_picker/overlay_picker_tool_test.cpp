// Copyright (c) 2014, JSK Lab
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <gmock/gmock.h>

#include "rviz_default_plugins/tools/overlay_picker/overlay_picker_tool.hpp"

using namespace ::testing;  // NOLINT

using rviz_default_plugins::tools::kShiftSnapGridSize;
using rviz_default_plugins::tools::snapToGrid;

TEST(OverlayPickerToolTest, snap_to_grid_leaves_exact_multiples_alone) {
  EXPECT_THAT(snapToGrid(0), Eq(0));
  EXPECT_THAT(snapToGrid(kShiftSnapGridSize), Eq(kShiftSnapGridSize));
  EXPECT_THAT(snapToGrid(-kShiftSnapGridSize), Eq(-kShiftSnapGridSize));
}

TEST(OverlayPickerToolTest, snap_to_grid_rounds_positive_values_down) {
  EXPECT_THAT(snapToGrid(1), Eq(0));
  EXPECT_THAT(snapToGrid(19), Eq(0));
  EXPECT_THAT(snapToGrid(21), Eq(kShiftSnapGridSize));
}

TEST(OverlayPickerToolTest, snap_to_grid_rounds_negative_values_towards_minus_infinity) {
  // Plain integer division truncates towards zero, which snaps -1 to 0 and
  // makes an overlay dragged past the origin jump a whole grid cell.
  EXPECT_THAT(snapToGrid(-1), Eq(-kShiftSnapGridSize));
  EXPECT_THAT(snapToGrid(-19), Eq(-kShiftSnapGridSize));
  EXPECT_THAT(snapToGrid(-21), Eq(-2 * kShiftSnapGridSize));
}

TEST(OverlayPickerToolTest, snap_to_grid_step_is_uniform_across_the_origin) {
  // Every step must be one grid cell; the truncating version produced a step
  // of zero between -19 and 1.
  for (int value = -3 * kShiftSnapGridSize; value < 3 * kShiftSnapGridSize; ++value) {
    const int difference = snapToGrid(value + kShiftSnapGridSize) - snapToGrid(value);
    EXPECT_THAT(difference, Eq(kShiftSnapGridSize)) << "at value " << value;
  }
}
