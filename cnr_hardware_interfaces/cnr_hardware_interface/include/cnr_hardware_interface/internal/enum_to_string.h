#include <string>

#undef DECL_ENUM_ELEMENT
#undef BEGIN_ENUM
#undef END_ENUM

#ifndef GENERATE_ENUM_STRINGS
    #define DECL_ENUM_ELEMENT( element ) element
    #define BEGIN_ENUM( ENUM_NAME ) typedef enum tag##ENUM_NAME
    #define END_ENUM( ENUM_NAME ) ENUM_NAME; \
            std::string to_string(enum tag##ENUM_NAME index)
#else
    #define DECL_ENUM_ELEMENT( element ) #element
    #define BEGIN_ENUM( ENUM_NAME ) std::string gs_##ENUM_NAME [] =
    #define END_ENUM( ENUM_NAME ) ; std::string to_string(enum tag##ENUM_NAME index){ return std::string( gs_##ENUM_NAME [index] ); }
#endif


#ifndef __ENUM__TO__STRING__H__
#define __ENUM__TO__STRING__H__

#include <type_traits>

namespace cnr_hardware_interface
{


template < typename C, C beginVal, C endVal>
class EnumIterator {
  typedef typename std::underlying_type<C>::type val_t;
  int val;
public:
  EnumIterator(const C & f) : val(static_cast<val_t>(f)) {}
  EnumIterator() : val(static_cast<val_t>(beginVal)) {}
  EnumIterator operator++() {
    ++val;
    return *this;
  }
  C operator*() { return static_cast<C>(val); }
  EnumIterator begin() { return *this; } //default ctor is good
  EnumIterator end() {
      static const EnumIterator endIter=++EnumIterator(endVal); // cache it
      return endIter;
  }
  bool operator!=(const EnumIterator& i) { return val != i.val; }
};


}
#endif
