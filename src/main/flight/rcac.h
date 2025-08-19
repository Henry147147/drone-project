#pragma once

struct pidProfile_s;
typedef struct pidProfile_s pidProfile_t;

void initRCACController(pidProfile_t *pidProfile);

void stepRCACController(const pidProfile_t *pidProfile);
