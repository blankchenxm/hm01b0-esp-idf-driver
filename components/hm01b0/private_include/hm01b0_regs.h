#ifndef HM01B0_REGS_H
#define HM01B0_REGS_H

/* Sensor identification (read-only). */
#define HM01B0_REG_MODEL_ID_H                   0x0000U
#define HM01B0_REG_MODEL_ID_L                   0x0001U
#define HM01B0_REG_SILICON_REV                  0x0002U
#define HM01B0_REG_FRAME_COUNT                  0x0005U
#define HM01B0_REG_PIXEL_ORDER                  0x0006U

#define HM01B0_PIXEL_ORDER_MASK                 0x03U
#define HM01B0_PIXEL_ORDER_GR                   0x00U
#define HM01B0_PIXEL_ORDER_RG                   0x01U
#define HM01B0_PIXEL_ORDER_BG                   0x02U
#define HM01B0_PIXEL_ORDER_GB                   0x03U

/* Sensor mode control. */
#define HM01B0_REG_MODE_SELECT                  0x0100U
#define HM01B0_REG_IMAGE_ORIENTATION            0x0101U
#define HM01B0_REG_SW_RESET                     0x0103U
#define HM01B0_REG_GROUP_PARAMETER_HOLD         0x0104U

/* Established HM01B0 drivers write 0x01 after changing CMU registers. */
#define HM01B0_GROUP_PARAMETER_APPLY             0x01U

#define HM01B0_MODE_SELECT_MASK                 0x07U
#define HM01B0_MODE_STANDBY                     0x00U
#define HM01B0_MODE_STREAMING                   0x01U
#define HM01B0_MODE_STREAMING_N_FRAMES          0x03U
#define HM01B0_MODE_HARDWARE_TRIGGER            0x05U

#define HM01B0_ORIENTATION_HMIRROR_MASK          0x01U
#define HM01B0_ORIENTATION_VFLIP_MASK            0x02U

/* Exposure and gain control. */
#define HM01B0_REG_INTEGRATION_H                0x0202U
#define HM01B0_REG_INTEGRATION_L                0x0203U
#define HM01B0_REG_ANALOG_GAIN                  0x0205U
#define HM01B0_REG_DIGITAL_GAIN_H               0x020EU
#define HM01B0_REG_DIGITAL_GAIN_L               0x020FU

/* Frame timing control. */
#define HM01B0_REG_FRAME_LENGTH_LINES_H         0x0340U
#define HM01B0_REG_FRAME_LENGTH_LINES_L         0x0341U
#define HM01B0_REG_LINE_LENGTH_PCK_H            0x0342U
#define HM01B0_REG_LINE_LENGTH_PCK_L            0x0343U
#define HM01B0_REG_DIGITAL_GAIN_CONTROL         0x0350U

/* Binning mode control. */
#define HM01B0_REG_READOUT_X                    0x0383U
#define HM01B0_REG_READOUT_Y                    0x0387U
#define HM01B0_REG_BINNING_MODE                 0x0390U

/* Test pattern control. */
#define HM01B0_REG_TEST_PATTERN_MODE            0x0601U
#define HM01B0_TEST_PATTERN_ENABLE_MASK         0x01U
#define HM01B0_TEST_PATTERN_SELECT_MASK         0x10U
#define HM01B0_TEST_PATTERN_REG_DISABLED        0x00U
#define HM01B0_TEST_PATTERN_REG_COLOR_BAR       0x01U
#define HM01B0_TEST_PATTERN_REG_WALKING_1       0x11U

/* Black level and defect pixel control. */
#define HM01B0_REG_BLC_CFG                      0x1000U
#define HM01B0_REG_BLC_DITHER                   0x1001U
#define HM01B0_REG_BLC_DARK_PIXEL_THRESHOLD    0x1002U
#define HM01B0_REG_BLC_TARGET                   0x1003U
#define HM01B0_REG_BLI_ENABLE                   0x1006U
#define HM01B0_REG_BLC2_TARGET                  0x1007U
#define HM01B0_REG_DPC_CTRL                     0x1008U
#define HM01B0_DPC_DISABLED                     0x00U
#define HM01B0_DPC_MONO_OPTION_1                0x01U
#define HM01B0_DPC_BAYER_OPTION_1               0x03U
#define HM01B0_DPC_BAYER_OPTION_2               0x05U
#define HM01B0_REG_SINGLE_THR_HOT               0x100BU
#define HM01B0_REG_SINGLE_THR_COLD              0x100CU
#define HM01B0_REG_SYNC_PIXEL_SHIFT_ENABLE      0x1012U

/* Statistics and motion-detection ROI. */
#define HM01B0_REG_STATISTIC_CTRL               0x2000U
#define HM01B0_REG_VENDOR_STAT_2003             0x2003U
#define HM01B0_REG_VENDOR_STAT_2004             0x2004U
#define HM01B0_REG_VENDOR_STAT_2007             0x2007U
#define HM01B0_REG_VENDOR_STAT_2008             0x2008U
#define HM01B0_REG_VENDOR_STAT_200B             0x200BU
#define HM01B0_REG_VENDOR_STAT_200C             0x200CU
#define HM01B0_REG_VENDOR_STAT_200F             0x200FU
#define HM01B0_REG_VENDOR_STAT_2010             0x2010U
#define HM01B0_REG_MD_LROI_X_START_H            0x2011U
#define HM01B0_REG_MD_LROI_X_START_L            0x2012U
#define HM01B0_REG_MD_LROI_Y_START_H            0x2013U
#define HM01B0_REG_MD_LROI_Y_START_L            0x2014U
#define HM01B0_REG_MD_LROI_X_END_H              0x2015U
#define HM01B0_REG_MD_LROI_X_END_L              0x2016U
#define HM01B0_REG_MD_LROI_Y_END_H              0x2017U
#define HM01B0_REG_MD_LROI_Y_END_L              0x2018U

/* Automatic exposure and gain control. */
#define HM01B0_REG_AE_CTRL                      0x2100U
#define HM01B0_REG_AE_TARGET_MEAN               0x2101U
#define HM01B0_REG_AE_MIN_MEAN                  0x2102U
#define HM01B0_REG_CONVERGE_IN_THRESHOLD        0x2103U
#define HM01B0_REG_CONVERGE_OUT_THRESHOLD       0x2104U
#define HM01B0_REG_MAX_INTEGRATION_H            0x2105U
#define HM01B0_REG_MAX_INTEGRATION_L            0x2106U
#define HM01B0_REG_MIN_INTEGRATION              0x2107U
#define HM01B0_REG_MAX_ANALOG_GAIN_FULL         0x2108U
#define HM01B0_REG_MAX_ANALOG_GAIN_BIN2         0x2109U
#define HM01B0_REG_MIN_ANALOG_GAIN              0x210AU
#define HM01B0_REG_MAX_DIGITAL_GAIN             0x210BU
#define HM01B0_REG_MIN_DIGITAL_GAIN             0x210CU
#define HM01B0_REG_DAMPING_FACTOR               0x210DU
#define HM01B0_REG_FLICKER_STEP_CTRL            0x210EU
#define HM01B0_REG_FLICKER_60HZ_H               0x210FU
#define HM01B0_REG_FLICKER_60HZ_L               0x2110U
#define HM01B0_REG_FLICKER_50HZ_H               0x2111U
#define HM01B0_REG_FLICKER_50HZ_L               0x2112U
#define HM01B0_REG_FLICKER_HYST_THRESHOLD       0x2113U

/* Motion detection control. */
#define HM01B0_REG_MD_CTRL                      0x2150U
#define HM01B0_REG_I2C_CLEAR                    0x2153U
#define HM01B0_REG_WMEAN_DIFF_THRESHOLD_H       0x2155U
#define HM01B0_REG_WMEAN_DIFF_THRESHOLD_M       0x2156U
#define HM01B0_REG_WMEAN_DIFF_THRESHOLD_L       0x2157U
#define HM01B0_REG_MD_THRESHOLD_H               0x2158U
#define HM01B0_REG_MD_THRESHOLD_M1              0x2159U
#define HM01B0_REG_MD_THRESHOLD_M2              0x215AU
#define HM01B0_REG_MD_THRESHOLD_L               0x215BU
#define HM01B0_REG_MD_INTERRUPT                 0x2160U

/* Sensor timing control. */
#define HM01B0_REG_QVGA_WINDOW_ENABLE           0x3010U
#define HM01B0_REG_SIX_BIT_MODE_ENABLE          0x3011U
#define HM01B0_REG_AUTOSLEEP_FRAME_COUNT        0x3020U
#define HM01B0_REG_ADVANCE_VSYNC                0x3022U
#define HM01B0_REG_ADVANCE_HSYNC                0x3023U
#define HM01B0_REG_EARLY_GAIN                   0x3035U

#define HM01B0_QVGA_ENABLE_MASK                 0x01U
#define HM01B0_RAW6_ENABLE_MASK                 0x01U

/* IO and clock control. */
#define HM01B0_REG_VENDOR_ANALOG_3044           0x3044U
#define HM01B0_REG_VENDOR_ANALOG_3045           0x3045U
#define HM01B0_REG_VENDOR_ANALOG_3047           0x3047U
#define HM01B0_REG_VENDOR_ANALOG_3050           0x3050U
#define HM01B0_REG_VENDOR_ANALOG_3051           0x3051U
#define HM01B0_REG_VENDOR_ANALOG_3052           0x3052U
#define HM01B0_REG_VENDOR_ANALOG_3053           0x3053U
#define HM01B0_REG_VENDOR_ANALOG_3054           0x3054U
#define HM01B0_REG_VENDOR_ANALOG_3055           0x3055U
#define HM01B0_REG_VENDOR_ANALOG_3056           0x3056U
#define HM01B0_REG_VENDOR_ANALOG_3057           0x3057U
#define HM01B0_REG_VENDOR_ANALOG_3058           0x3058U
#define HM01B0_REG_BIT_CONTROL                  0x3059U
#define HM01B0_REG_OSC_CLK_DIV                  0x3060U
#define HM01B0_REG_ANA_REGISTER_11              0x3061U
#define HM01B0_REG_IO_DRIVE_STRENGTH            0x3062U
#define HM01B0_REG_IO_DRIVE_STRENGTH_2          0x3063U
#define HM01B0_REG_ANA_REGISTER_14              0x3064U
#define HM01B0_REG_OUTPUT_PIN_STATUS_CONTROL    0x3065U
#define HM01B0_REG_ANA_REGISTER_17              0x3067U
#define HM01B0_REG_PCLK_POLARITY                0x3068U

#define HM01B0_INTERFACE_WIDTH_MASK             0x60U
#define HM01B0_INTERFACE_8_BIT                  0x00U
#define HM01B0_INTERFACE_4_BIT                  0x40U
#define HM01B0_INTERFACE_1_BIT                  0x20U

/* OSC_CLK_DIV[1:0]: Sensor_Core clock divider. */
#define HM01B0_CORE_DIVIDER_MASK                0x03U
#define HM01B0_CORE_DIVIDE_BY_8                 0x00U
#define HM01B0_CORE_DIVIDE_BY_4                 0x01U
#define HM01B0_CORE_DIVIDE_BY_2                 0x02U
#define HM01B0_CORE_DIVIDE_BY_1                 0x03U

/* OSC_CLK_DIV[3:2]: Sensor_Register internal clock divider. */
#define HM01B0_SENSOR_REGISTER_DIVIDER_MASK     0x0CU
#define HM01B0_SENSOR_REGISTER_DIVIDE_BY_4      0x00U
#define HM01B0_SENSOR_REGISTER_DIVIDE_BY_8      0x04U
#define HM01B0_SENSOR_REGISTER_DIVIDE_BY_1      0x08U
#define HM01B0_SENSOR_REGISTER_DIVIDE_BY_2      0x0CU
#define HM01B0_NIBBLE_BIT_ORDER_MASK            0x10U
#define HM01B0_NIBBLE_LSB_FIRST                 0x00U
#define HM01B0_NIBBLE_MSB_FIRST                 0x10U
#define HM01B0_PCLK_GATED_ENABLE_MASK           0x20U

#define HM01B0_TRIGGER_SYNC_ENABLE_MASK         0x04U

/*
 * V05 section 6.4.1 and the established Himax initialization sequence use 0
 * when switching from the self oscillator to an applied external MCLK. The
 * wording in the V05 register table describes this bit in the opposite way,
 * so this setting must be confirmed on hardware before dynamic clock switch.
 */
#define HM01B0_CLOCK_SOURCE_MASK                0x01U
#define HM01B0_CLOCK_SOURCE_EXTERNAL_MCLK       0x00U
#define HM01B0_TRIGGER_MCLK_EDGE_MASK           0x02U

#define HM01B0_PCLK_POLARITY_MASK               0x01U
#define HM01B0_PCLK_RISING_EDGE                 0x00U
#define HM01B0_PCLK_FALLING_EDGE                0x01U

/* Software reset accepts either 0 or 1; use 1 consistently. */
#define HM01B0_SOFTWARE_RESET                   0x01U

/* Programmable I2C slave address. */
#define HM01B0_REG_I2C_ID_SELECT                0x3400U
#define HM01B0_REG_I2C_ID                       0x3401U

#endif /* HM01B0_REGS_H */
