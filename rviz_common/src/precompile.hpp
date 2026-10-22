// Copyright (c) 2026, Open Source Robotics Foundation, Inc.
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

// Build-only precompiled header, not part of the public API and not
// installed. Contents are limited to headers profiling (clang -ftime-trace)
// showed being parsed with near-identical, non-template-instantiation cost
// in the large majority of this package's translation units: Qt6 and a few
// expensive-to-parse standard headers.
//
// Also includes four of rviz_common's own base-class headers that showed up
// heavy in the same profile (property.hpp, display.hpp, bool_property.hpp,
// panel.hpp). This is safe here because rviz_common builds as a single
// target: every translation unit that includes this PCH shares identical
// compile definitions, so there's no risk of the GCC -Winvalid-pch
// macro-state mismatch that ruled out REUSE_FROM for rviz_default_plugins'
// multi-target PCH.

#include <chrono>
#include <format>
#include <functional>
#include <memory>
#include <string>

#include <QAction>
#include <QComboBox>
#include <QDebug>
#include <QFrame>
#include <QIcon>
#include <QObject>
#include <QPixmap>
#include <QString>
#include <QVariant>
#include <QWidget>

#include "rviz_common/display.hpp"
#include "rviz_common/panel.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/property.hpp"
