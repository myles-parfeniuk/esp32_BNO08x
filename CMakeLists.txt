# `driver` was a single component pre-v5.2; it kept gpio/spi inside
# and transitively pulled them via REQUIRES. From v5.2 onwards the
# per-peripheral split (`esp_driver_gpio`, `esp_driver_spi`) became
# canonical, and v6 tightens the rules so missing direct REQUIRES
# surface as build errors. Keep both spellings supported by adding
# the split components only when the running IDF is v5.2 or newer.
set(reqs driver esp_timer freertos log cmock)
if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_GREATER_EQUAL "5.2")
    list(APPEND reqs esp_driver_gpio esp_driver_spi)
endif()

idf_component_register(SRC_DIRS "source"  "source/report" "SH2"
                    INCLUDE_DIRS "." "include" "include/report" "include/callback" "SH2"
                    REQUIRES ${reqs})
