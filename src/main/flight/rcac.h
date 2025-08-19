#pragma once

#include "pid.h"

void runRCACController(pidProfile_t *pidProfile, timeUs_t currentTimeUs);

// TODO figure out if this is needed
void initRCACController(pidProfile_t *pidProfile);