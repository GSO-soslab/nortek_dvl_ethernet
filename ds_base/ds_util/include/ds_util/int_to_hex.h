/**
* Copyright 2018 Woods Hole Oceanographic Institution
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
*    may be used to endorse or promote products derived from this software
*    without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/
#ifndef DS_UTIL_INT2HEX_H
#define DS_UTIL_INT2HEX_H

#include <string>
#include <sstream>
#include <iomanip>
#include <stdint.h>

namespace ds_util
{
template <typename T1>
static std::string int_to_hex(T1 i)
{
  uint16_t j = i;  // Recast to uint16 so that it ill be treated as an integer, not a char
  j &= 0x00FF;     // Ensure that the integer remains within uint8 range
  std::stringstream stream;
  stream << std::uppercase << std::setfill('0') << std::setw(sizeof(uint8_t) * 2) << std::hex << j;
  return stream.str();
}

template <typename T2>
static std::string int_to_32_hex(T2 i)
{
  uint16_t j;
  int s = sizeof(uint32_t) * 2;
  std::stringstream stream;
  for (int it = 1; it <= s; it++)
  {
    j = i >> (s - it) * 4;  // Recast to uint16 so that it ill be treated as an integer, not a char
    j &= 0x000F;            // Ensure that the integer remains within uint4 range
    stream << std::uppercase << std::setfill('0') << std::setw(sizeof(uint8_t)) << std::hex << j;
  }

  return stream.str();
}

template <typename T3>
static std::string int_to_long_hex(T3 i)
{
  uint16_t j;
  int s = sizeof(i);
  std::stringstream stream;
  for (int it = 1; it <= s; it++)
  {
    j = i >> (s - it) * 4;  // Recast to uint16 so that it ill be treated as an integer, not a char
    j &= 0x000F;            // Ensure that the integer remains within uint8 range
    stream << std::uppercase << std::setfill('0') << std::setw(sizeof(uint8_t)) << std::hex << j;
  }

  return stream.str();
}
}

#endif  // DS_UTIL_INT2HEX_H
