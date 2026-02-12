#ifndef RCTOPOSITION_H
#define RCTOPOSITION_H

#include <cstdint>
#include "io/rc_controls.h"

extern int16_t rcCommand[4];
extern float posTargetZ_m;

void checkUserCommands();

#endif
