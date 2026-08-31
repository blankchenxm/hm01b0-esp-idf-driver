#ifndef HM01B0_TABLES_H
#define HM01B0_TABLES_H

#include <stddef.h>

#include "hm01b0_types.h"

extern const hm01b0_regval_t hm01b0_common_init[];
extern const size_t hm01b0_common_init_count;

extern const hm01b0_regval_t hm01b0_variant_mono[];
extern const size_t hm01b0_variant_mono_count;
extern const hm01b0_regval_t hm01b0_variant_bayer[];
extern const size_t hm01b0_variant_bayer_count;

extern const hm01b0_regval_t hm01b0_mode_full[];
extern const size_t hm01b0_mode_full_count;
extern const hm01b0_regval_t hm01b0_mode_qvga[];
extern const size_t hm01b0_mode_qvga_count;
extern const hm01b0_regval_t hm01b0_mode_qqvga[];
extern const size_t hm01b0_mode_qqvga_count;

extern const hm01b0_regval_t hm01b0_interface_8bit[];
extern const size_t hm01b0_interface_8bit_count;
extern const hm01b0_regval_t hm01b0_interface_4bit[];
extern const size_t hm01b0_interface_4bit_count;
extern const hm01b0_regval_t hm01b0_interface_1bit[];
extern const size_t hm01b0_interface_1bit_count;

extern const hm01b0_regval_t hm01b0_test_pattern_off[];
extern const size_t hm01b0_test_pattern_off_count;
extern const hm01b0_regval_t hm01b0_test_pattern_color_bar[];
extern const size_t hm01b0_test_pattern_color_bar_count;
extern const hm01b0_regval_t hm01b0_test_pattern_walking_1[];
extern const size_t hm01b0_test_pattern_walking_1_count;

#endif /* HM01B0_TABLES_H */
