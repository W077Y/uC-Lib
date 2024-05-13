#include <uC_Errors.hpp>

namespace uC::Errors
{
  void uC_config_error(char const*)
  {
    __asm("bkpt 255");
    __asm("bx lr");
  }

  void not_implemented()
  {
    __asm("bkpt 255");
    __asm("bx lr");
  }

  void invalid_hw_unit()
  {
    __asm("bkpt 255");
    __asm("bx lr");
  }

  void multiple_use()
  {
    __asm("bkpt 255");
    __asm("bx lr");
  }
}    // namespace uC::Errors
