
#if ( !defined(__CNR__ROBOT__HW__STATUS__H__) || defined(GENERATE_ENUM_STRINGS) )

#if (!defined(GENERATE_ENUM_STRINGS))
    #define __CNR__ROBOT__HW__STATUS__H__
#endif

#include <cnr_hardware_interface/internal/enum_to_string.h>


namespace cnr_hardware_interface
{

// ENUM: it defines both the enum, and a global variable (array), that sotres the string corresponding to each variable, imposed as '#VARAIBLE' using preprocessor directive. A function to_string( StatusHw ) is also defined.
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

}

#endif
