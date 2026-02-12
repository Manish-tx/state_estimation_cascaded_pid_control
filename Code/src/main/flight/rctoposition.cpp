#include "rctoposition.h"
#include "io/rc_controls.h"
#include "rx/rx.h"
// RC command handler.
// Call this inside your main control loop.
void checkUserCommands()
{
    // If rcommand[0] button/switch is pressed
    if (rcCommand[0] >=10)
    {
        posTargetZ_m = 70;  // Set desired Z to 50 cm
    }
}
