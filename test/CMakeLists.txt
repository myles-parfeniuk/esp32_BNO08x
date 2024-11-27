idf_component_register(SRC_DIRS "."
                    INCLUDE_DIRS "."
                    REQUIRES cmock esp32_BNO08x)

# supress unused var warnings from test_imu
idf_build_set_property(COMPILE_OPTIONS "-Wno-unused-but-set-variable" APPEND)