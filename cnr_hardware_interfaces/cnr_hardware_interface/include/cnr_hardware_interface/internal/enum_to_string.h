/*
 *  Software License Agreement (New BSD License)
 *
 *  Copyright 2020 National Council of Research of Italy (CNR)
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
#include <string>

#undef DECL_ENUM_ELEMENT
#undef BEGIN_ENUM
#undef END_ENUM

#ifndef GENERATE_ENUM_STRINGS
#define DECL_ENUM_ELEMENT(element) element
#define BEGIN_ENUM(ENUM_NAME) typedef enum tag##ENUM_NAME
#define END_ENUM(ENUM_NAME) ENUM_NAME; \
            std::string to_string(enum tag##ENUM_NAME index)
#else
#define DECL_ENUM_ELEMENT(element) #element
#define BEGIN_ENUM(ENUM_NAME) std::string gs_##ENUM_NAME [] =
#define END_ENUM(ENUM_NAME) ; \
            std::string to_string(enum tag##ENUM_NAME index) { return std::string(gs_##ENUM_NAME[index]); }
#endif


#ifndef  CNR_HARDWARE_INTERFACE_INTERNAL_ENUM_TO_STRING_H
#define  CNR_HARDWARE_INTERFACE_INTERNAL_ENUM_TO_STRING_H

#include <type_traits>

namespace cnr_hardware_interface
{


template < typename C, C beginVal, C endVal>
class EnumIterator
{
  typedef typename std::underlying_type<C>::type val_t;
  int val;
public:
  EnumIterator() : val(static_cast<val_t>(beginVal)) {}
  explicit EnumIterator(const C & f) : val(static_cast<val_t>(f)) {}
  EnumIterator operator++()
  {
    ++val;
    return *this;
  }
  C operator*()
  {
    return static_cast<C>(val);
  }
  EnumIterator begin()
  {
    return *this;  // default ctor is good
  }
  EnumIterator end()
  {
    static const EnumIterator endIter = ++EnumIterator(endVal);  // cache it
    return endIter;
  }
  bool operator!=(const EnumIterator& i)
  {
    return val != i.val;
  }
};

}  // namespace cnr_hardware_interface

#endif  // CNR_HARDWARE_INTERFACE_INTERNAL_ENUM_TO_STRING_H
