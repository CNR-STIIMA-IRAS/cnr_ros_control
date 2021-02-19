#ifndef CNR_HARDWARE_INTERFACE__VECTOR_TO_STRING__H
#define CNR_HARDWARE_INTERFACE__VECTOR_TO_STRING__H

#include <vector>
#include <string>

namespace cnr_hardware_interface
{

template< typename T >
inline std::string to_string(const std::vector< T >& what)
{
  std::string ret = "< ";
  for (const auto & w : what) ret += std::to_string(w) + " ";
  ret += " >";
  return ret;
}

template<>
inline std::string to_string(const std::vector<std::string>& what)
{
  std::string ret = "["+std::to_string(what.size()) + "][";
  for (const auto & w : what) ret += w + ",";
  ret += "]";
  return ret;
}

template< typename T >
inline std::string to_string(const T& what)
{
  std::string ret = "< " + std::to_string(what) + " >";
  return ret;
}

template< >
inline std::string to_string(const std::string& what)
{
  std::string ret = "< " + what + " >";
  return ret;
}

}  // namespace cnr_hardware_interface


#endif // CNR_HARDWARE_INTERFACE__VECTOR_TO_STRING__H
