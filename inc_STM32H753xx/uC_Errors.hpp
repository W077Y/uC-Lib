#pragma once
#ifndef UC_ERRORS_HPP
#define UC_ERRORS_HPP

namespace uC
{
  namespace Errors
  {
    void uC_config_error(char const* msg);
    void not_implemented();
    void invalid_hw_unit();
    void multiple_use();
  }    // namespace Errors
}    // namespace uC

#endif
