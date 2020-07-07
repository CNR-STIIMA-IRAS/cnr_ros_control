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
#ifndef __CNR_CONTROLLER_MANAGER_INTERFACE__H__
#define __CNR_CONTROLLER_MANAGER_INTERFACE__H__


namespace cnr_controller_manager_interface
{



template< typename T >
bool unique(std::vector< T >& vv)
{
  for (size_t i = 0; i < vv.size(); i++)
  {
    std::string check = vv[i];
    for (size_t j = i + 1; j < vv.size(); j++)
    {
      if (check == vv[j])
      {
        vv.erase(vv.begin() + j);
      }
    }
  }
  return true;
}


template< typename T >
bool equal(const std::vector< T >& lhs, const std::vector< T >& rhs)
{
  if (lhs.size() != rhs.size()) return false;
  for (auto const l : lhs)
  {
    if (std::find(rhs.begin(), rhs.end(), l) == rhs.end()) return false;
  }
  return true;
}

template< typename T >
void extract(const std::vector< T >& va
             , const std::vector< T >& vb
             , std::vector< T >* a_not_in_b = nullptr
             , std::vector< T >* b_not_in_a = nullptr
             , std::vector< T >* a_in_b     = nullptr)
{
  if (a_not_in_b) a_not_in_b->clear();
  if (b_not_in_a) b_not_in_a->clear();
  if (a_in_b) a_in_b    ->clear();

  if ((va.size() == 0) && (vb.size() == 0))
  {
    return;
  }
  else if (va.size() == 0)
  {
    if (a_not_in_b) a_not_in_b   ->clear();
    if (b_not_in_a) *b_not_in_a = vb;
    if (a_in_b) a_in_b      ->clear();
  }
  else if (vb.size() == 0)
  {
    if (a_not_in_b) *a_not_in_b = va;
    if (b_not_in_a) b_not_in_a  -> clear();
    if (a_in_b) a_in_b      ->clear();
  }
  else
  {
    for (auto const a : va)
    {
      if (std::find(vb.begin(), vb.end(), a) == vb.end())
      {
        if (a_not_in_b) a_not_in_b->push_back(a);
      }
      else
      {
        if (a_in_b) a_in_b->push_back(a);
      }
    }
    for (auto const b : vb)
    {
      if (std::find(va.begin(), va.end(), b) == va.end())
      {
        if (b_not_in_a) b_not_in_a->push_back(b);
      }
    }
  }
}

template< class T >
std::string to_string(const std::vector< T >& what)
{
  std::string ret = "<";
  for (size_t j = 0; j < what.size(); j++)
  {
    ret += std::to_string(what.at(j)) + (j < what.size() - 1 ? ", " : "");
  }
  ret += ">";
  return ret;
}


inline
std::string to_string(const double& what)
{
  std::string ret = "<"  + std::to_string(what) + ">";
  return ret;
}


inline
std::string to_string(const std::vector<std::string>& what, const std::string value_header = "ctrl")
{
  std::string ret = value_header + ": <";
  for (size_t j = 0; j < what.size(); j++)
  {
    ret += what.at(j) + (j < what.size() - 1 ? ", " : "");
  }
  ret += ">";
  return ret;
}


struct RetrieveKey
{
  template <typename T>
  typename T::first_type operator()(T keyValuePair) const
  {
    return keyValuePair.first;
  }
};

//============ UTILITIES
template<class MSG>
bool callRequest(ros::ServiceClient& clnt, MSG& msg, std::string& error, const ros::Duration&  watchdog = ros::Duration(0.0))
{
  if (watchdog.toSec() > 0)
  {
    if (!clnt.waitForExistence(watchdog))
    {
      error = "The service '" + clnt.getService() + "' does not exist. Abort." ;
      return false;
    }
  }
  if (!clnt.call(msg))
  {
    error = "The service '" + clnt.getService() + "' is broken. Abort.";
    std::cout << msg.request << std::endl;
    return false;
  }
  return true;
}



}

#endif
