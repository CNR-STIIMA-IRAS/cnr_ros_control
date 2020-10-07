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
#if (!defined(CNR_HARDWARE_INTERFACE_INTERNAL_CNR_ROBOT_HW_STATUS_H) || defined(GENERATE_ENUM_STRINGS))  // NOLINT(build/include)

#if (!defined(GENERATE_ENUM_STRINGS))
#define CNR_HARDWARE_INTERFACE_INTERNAL_CNR_ROBOT_HW_STATUS_H
#endif

#include <cnr_hardware_interface/internal/enum_to_string.h>


namespace cnr_hardware_interface
{

// ENUM: it defines both the enum, and a global variable (array), that sotres the string corresponding to each variable
//        imposed as '#VARAIBLE' using preprocessor directive. A function to_string( StatusHw ) is also defined.
BEGIN_ENUM(StatusHw)
{
  DECL_ENUM_ELEMENT(UNLOADED),
                    DECL_ENUM_ELEMENT(CREATED),
                    DECL_ENUM_ELEMENT(INITIALIZED),
                    DECL_ENUM_ELEMENT(READY),
                    DECL_ENUM_ELEMENT(RUNNING),
                    DECL_ENUM_ELEMENT(SHUTDOWN),
                    DECL_ENUM_ELEMENT(ERROR),
                    DECL_ENUM_ELEMENT(CTRL_ERROR),
                    DECL_ENUM_ELEMENT(SRV_ERROR)
}
END_ENUM(StatusHw);

}  // namespace cnr_hardware_interface

#endif  // !defined(CNR_HARDWARE_INTERFACE_INTERNAL_CNR_ROBOT_HW_STATUS_H) || defined(GENERATE_ENUM_STRINGS)
