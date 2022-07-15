/*
 * Copyright (c) 2018, Bosch Software Innovations GmbH.
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
 *     * Neither the name of the copyright holder nor the names of its contributors
 *       may be used to endorse or promote products derived from
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

#include <gmock/gmock.h>

#include <OgreMatrix3.h>
#include <OgreQuaternion.h>
#include <OgreVector.h>

#include "../../../../src/rviz_default_plugins/displays/odometry/quaternion_helper.hpp"

using namespace ::testing;  // NOLINT

TEST(QuaternionHelper, ogreQuaternionHelper_returns_0_for_equal_inputs) {
  Ogre::Quaternion quaternion(1.0f, 0.0f, 0.0f, 0.0f);

  EXPECT_THAT(
    rviz_default_plugins::ogreQuaternionAngularDistance(quaternion, quaternion), Eq(0.0f));
}

TEST(QuaternionHelper, ogreQuaternionHelper_returns_angle) {
  Ogre::Vector3 axis(1, 2, 3);
  axis.normalise();
  Ogre::Quaternion quaternion1;
  quaternion1.FromAngleAxis(Ogre::Radian(0), axis);
  Ogre::Quaternion quaternion2;
  quaternion2.FromAngleAxis(Ogre::Radian(0.5f), axis);
  Ogre::Quaternion quaternion3;
  quaternion3.FromAngleAxis(Ogre::Radian(1.2f), axis);

  EXPECT_THAT(
    rviz_default_plugins::ogreQuaternionAngularDistance(quaternion1, quaternion2),
    FloatNear(0.5f, 0.001f));
  EXPECT_THAT(
    rviz_default_plugins::ogreQuaternionAngularDistance(quaternion1, quaternion3),
    FloatNear(1.2f, 0.001f));
}
