set(srcs
  src/bitwise_utils.c
  src/code_extraction.c
  src/localization_loop.c
  src/location_decode.c
  src/phase_correlation.c
  src/mls_index.c
  src/mls_query.c
  src/micro_image_utils/image_utils.c

# customized to use esp32 optimized fft
  #src/micro_image_utils/mfft.c
  fft.c

# for tests only
  src/debug_prints.c
  src/micro_image_utils/distance_transform.c
  src/tests/test_bitwise_utils.c
  src/tests/test_code_extraction.c
  src/tests/test_location_decode.c
  src/tests/test_localization_loop.c
  src/tests/test_mls_query.c
  src/micro_image_utils/tests/test_generic_utils.c
  src/micro_image_utils/tests/test_image_utils.c
  src/micro_image_utils/tests/test_math_utils.c
)

set(includes
  include
  include/micro_image_utils
  include/micro_image_utils/tests
)

idf_component_register(
  SRCS ${srcs}
  INCLUDE_DIRS ${includes}
  PRIV_REQUIRES esp-dsp
)

target_compile_options(${COMPONENT_LIB} PRIVATE
  -Wall
  -Wextra
  -Werror=double-promotion
  -Ofast
)
