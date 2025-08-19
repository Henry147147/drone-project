/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

extern "C" {
#include "platform.h"
#include "build/debug.h"
#include "flight/pid.h"
#include "flight/rcac.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

int16_t debug[DEBUG16_VALUE_COUNT];
uint8_t debugMode;

class RcacControllerTest : public ::testing::Test {
protected:
    pidProfile_t pidProfile;

    void SetUp() override {
        memset(&pidProfile, 0, sizeof(pidProfile));

        for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
            pidProfile.RCAC_hyperparameters[i] = (RCAC_hyperparameters_t){
                .reg_z = false,
                .r0 = 0.0f,
                .rz = 0.0f,
                .lambda = 0.0f,
                .nc = 3,
                .window = 0,
                .reg_size = 3,
                .FILT_nf = 0,
                .FILT_nu = 0.0f,
            };

            pidProfile.RCAC_input_output[i] = (RCAC_input_output_t){
                .k = 0,
                .u = 0.0f,
                .z = 0.0f,
                .yp = 0.0f,
                .r = 0.0f,
            };
        }

        pidProfile.RCAC_input_output[PID_ROLL].u = 10.0f;
        pidProfile.RCAC_input_output[PID_ROLL].z = 20.0f;
        pidProfile.RCAC_input_output[PID_ROLL].yp = 30.0f;
        pidProfile.RCAC_input_output[PID_ROLL].r = 40.0f;
    }

    void TearDown() override {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            RCAC_internal_state_t *state = &pidProfile.RCAC_internal_state[axis];
            free(state->u_h);
            free(state->z_h);
            free(state->r_h);
            free(state->yp_h);
            if (state->PHI_window) {
                for (int i = 0; i < pidProfile.RCAC_hyperparameters[axis].reg_size; ++i) {
                    free(state->PHI_window[i]);
                }
                free(state->PHI_window);
            }
            free(state->u_window);
            free(state->z_window);
            if (state->PHI_filt_window) {
                for (int i = 0; i < pidProfile.RCAC_hyperparameters[axis].reg_size; ++i) {
                    free(state->PHI_filt_window[i]);
                }
                free(state->PHI_filt_window);
            }
            free(state->u_filt_window);
            free(state->PHI);
        }
    }
};

TEST_F(RcacControllerTest, UpdatesHistoryBuffers) {
    runRCACController(&pidProfile, 0);

    RCAC_internal_state_t *state = &pidProfile.RCAC_internal_state[PID_ROLL];

    ASSERT_NE(state->u_h, nullptr);
    ASSERT_NE(state->z_h, nullptr);
    ASSERT_NE(state->r_h, nullptr);
    ASSERT_NE(state->yp_h, nullptr);
    ASSERT_NE(state->PHI, nullptr);

    EXPECT_FLOAT_EQ(10.0f, state->u_h[0]);
    EXPECT_FLOAT_EQ(0.0f, state->u_h[1]);

    EXPECT_FLOAT_EQ(20.0f, state->z_h[0]);
    EXPECT_FLOAT_EQ(0.0f, state->z_h[1]);
    EXPECT_FLOAT_EQ(0.0f, state->z_h[2]);

    EXPECT_FLOAT_EQ(40.0f, state->r_h[0]);

    EXPECT_FLOAT_EQ(30.0f, state->yp_h[0]);

    EXPECT_FLOAT_EQ(20.0f, state->intg);

    EXPECT_FLOAT_EQ(30.0f, state->PHI[0]);
    EXPECT_FLOAT_EQ(20.0f, state->PHI[1]);
    EXPECT_FLOAT_EQ(30.0f, state->PHI[2]);

    pidProfile.RCAC_input_output[PID_ROLL].u = 11.0f;
    pidProfile.RCAC_input_output[PID_ROLL].z = 21.0f;
    pidProfile.RCAC_input_output[PID_ROLL].yp = 31.0f;
    pidProfile.RCAC_input_output[PID_ROLL].r = 41.0f;

    runRCACController(&pidProfile, 0);

    EXPECT_FLOAT_EQ(11.0f, state->u_h[0]);
    EXPECT_FLOAT_EQ(10.0f, state->u_h[1]);

    EXPECT_FLOAT_EQ(21.0f, state->z_h[0]);
    EXPECT_FLOAT_EQ(20.0f, state->z_h[1]);
    EXPECT_FLOAT_EQ(0.0f, state->z_h[2]);

    EXPECT_FLOAT_EQ(41.0f, state->r_h[0]);
    EXPECT_FLOAT_EQ(40.0f, state->r_h[1]);

    EXPECT_FLOAT_EQ(31.0f, state->yp_h[0]);
    EXPECT_FLOAT_EQ(30.0f, state->yp_h[1]);

    EXPECT_FLOAT_EQ(41.0f, state->intg);

    EXPECT_FLOAT_EQ(31.0f, state->PHI[0]);
    EXPECT_FLOAT_EQ(41.0f, state->PHI[1]);
    EXPECT_FLOAT_EQ(1.0f, state->PHI[2]);
}