// Copyright (c) 2017, Bosch Software Innovations GmbH.
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


#include "string_helper.hpp"

#include <algorithm>
#include <string>
#include <vector>

// Trim only space, tab, and newline. Carriage return (\r) and form feed (\f) are
// intentionally excluded so they are preserved in CMake-derived strings instead of
// being silently stripped. The set is enumerated explicitly rather than derived from
// std::isspace(c, std::locale("")) because that constructs a locale per character,
// makes the result depend on the (possibly misconfigured) system locale, and can throw
// std::runtime_error when that locale is invalid.
static bool is_trim_whitespace(char character)
{
  return character == ' ' || character == '\t' || character == '\n';
}

// Used to parse strings derived from CMake
// using the system locale, since CMake seems to take the system locale to parse strings
std::vector<std::string> rviz_rendering::string_helper::splitStringIntoTrimmedItems(
  const std::string & string_to_split, const char delimiter)
{
  std::stringstream stringstream(string_to_split);
  std::string item;
  std::vector<std::string> filenames;
  while (std::getline(stringstream, item, delimiter)) {
    if (std::all_of(item.begin(), item.end(), is_trim_whitespace)) {
      item.clear();
    } else {
      auto whitespace_front = std::find_if_not(item.begin(), item.end(), is_trim_whitespace);
      auto whitespace_back = std::find_if_not(item.rbegin(), item.rend(), is_trim_whitespace);

      // Only perform erase operation if the string is not empty
      if (whitespace_front != item.end() && whitespace_back != item.rend()) {
        item.erase(whitespace_back.base(), item.end());
        item.erase(item.begin(), whitespace_front);
      }
      if (!item.empty()) {
        filenames.push_back(item);
      }
    }
  }
  return filenames;
}
