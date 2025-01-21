#pragma once
#include <math.h>

/**
 * @brief A macro to define a parameter
 *
 * This macro is used to define parameters. It takes the name of the variable,
 * the name of the parameter, the type of the parameter, the default value of
 * the parameter and a description of the parameter.
 *
 * @param vname The name of the variable
 * @param stname The name of the parameter
 * @param ptype The type of the parameter
 * @param default The default value of the parameter
 * @param stdescription The description of the parameter
 */
#define PARAM_DEF(vname, stname, ptype, default, stdescription)    \
  {                                                                \
    rcl_interfaces::msg::ParameterDescriptor __tmp_d__;            \
    __tmp_d__.name = stname;                                       \
    __tmp_d__.type = ptype;                                        \
    __tmp_d__.description = stdescription;                         \
    vname = declare_parameter(__tmp_d__.name, default, __tmp_d__); \
  }

/**
 * @brief A macro to define a double parameter
 *
 * This macro is used to define a double parameter. It takes the name of the
 * variable, the name of the parameter, the default value of the parameter and
 * a description of the parameter.
 *
 * @param vname The name of the variable
 * @param stname The name of the parameter
 * @param default The default value of the parameter
 * @param stdescription The description of the parameter
 */
#define PARAM_DOUBLE(vname, stname, default, stdescription)                \
  PARAM_DEF(vname, stname,                                                 \
            rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE, default, \
            stdescription)

/**
 * @brief A macro to define an int parameter
 *
 * This macro is used to define an int parameter. It takes the name of the
 * variable, the name of the parameter, the default value of the parameter and
 * a description of the parameter.
 *
 * @param vname The name of the variable
 * @param stname The name of the parameter
 * @param default The default value of the parameter
 * @param stdescription The description of the parameter
 */
#define PARAM_INT(vname, stname, default, stdescription)                    \
  PARAM_DEF(vname, stname,                                                  \
            rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER, default, \
            stdescription)

#define PARAM_STR(vname, stname, default, stdescription)                   \
  PARAM_DEF(vname, stname,                                                 \
            rcl_interfaces::msg::ParameterType::PARAMETER_STRING, default, \
            stdescription)

namespace common {
/**
 * @brief Convert radians to degrees
 * @param r radians
 * @return degrees
 */
float inline rtod(float r) { return r * 180 / M_PI; }

/**
 * @brief Convert degrees to radians
 * @param r degrees
 * @return radians
 */
float inline dtor(float r) { return r * M_PI / 180.0; }
}  // namespace common
