#pragma once

struct pidProfile_s;
typedef struct pidProfile_s pidProfile_t;

void runRCACController(pidProfile_t *pidProfile, timeUs_t currentTimeUs);

// TODO figure out if this is needed
void initRCACController(pidProfile_t *pidProfile);