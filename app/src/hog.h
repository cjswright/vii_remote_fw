/** @file
 *  @brief HoG Service sample
 */

/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifdef __cplusplus
extern "C" {
#endif

void hog_init(void);

void hog_button_loop(void);


void
hog_update_xy(int8_t x, int8_t y);


#ifdef __cplusplus
}
#endif
