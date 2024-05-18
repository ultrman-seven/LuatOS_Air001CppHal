#ifndef D104B25D_7A0B_4D98_A25F_E3D1B0361649
#define D104B25D_7A0B_4D98_A25F_E3D1B0361649

#include "stdint.h"
namespace sys
{
    void delayMs(uint32_t ms);
    void delayBreak();
    uint32_t getSysClkFreqHz();
    uint32_t getHClkFreqHz();
    uint32_t getPClkFreqHz();
} // namespace sys

#endif/* D104B25D_7A0B_4D98_A25F_E3D1B0361649 */
