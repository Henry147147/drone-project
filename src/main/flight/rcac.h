#pragma once

#include "pid.h"

void initRCACController(const pidProfile_t *pidProfile);

void stepRCACController(const pidProfile_t *pidProfile);