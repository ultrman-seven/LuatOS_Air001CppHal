#include "wySys.hpp"

uint64_t __msTimeStamp = 0;
uint32_t __msDelayCnt = 0;

void SysTick_Handler(void)
{
    ++__msTimeStamp;
    if (__msDelayCnt)
        --__msDelayCnt;
}

void sysInit()
{
}

namespace sys
{

} // namespace sys
