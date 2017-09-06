/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * Copyright (c) 2017, Open Source Robotics Foundation, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef RVIZ_RENDERING__CUSTOM_PARAMETER_INDICES_HPP_
#define RVIZ_RENDERING__CUSTOM_PARAMETER_INDICES_HPP_

// These are custom parameter indexes for the shader programs.  Keep
// them all here, so they stay consistent across the app.
//
// These need to agree with the values in the shader programs, which
// are defined here: ogre_media/materials/glsl/*.program.
// In there, look for lines like:
//   param_named_auto <param name> custom <index number>
// They are spread out across the files, appearing just
// where they are needed.
#define RVIZ_RENDERING_SIZE_PARAMETER 0
#define RVIZ_RENDERING_ALPHA_PARAMETER 1
#define RVIZ_RENDERING_PICK_COLOR_PARAMETER 2
#define RVIZ_RENDERING_NORMAL_PARAMETER 3
#define RVIZ_RENDERING_UP_PARAMETER 4
#define RVIZ_RENDERING_HIGHLIGHT_PARAMETER 5
#define RVIZ_RENDERING_AUTO_SIZE_PARAMETER 6

#endif  // RVIZ_RENDERING__CUSTOM_PARAMETER_INDICES_HPP_
