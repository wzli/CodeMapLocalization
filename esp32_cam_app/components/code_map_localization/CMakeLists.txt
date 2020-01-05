set(srcs
  src/code_extraction.c
  src/location_decode.c
  src/mls_index.c
  src/mls_query.c
  src/utils/bitwise_utils.c
  src/utils/image_utils.c

#for tests
  src/debug_prints.c
  src/tests/test_math_utils.c
  src/tests/test_image_utils.c
  src/tests/test_bitwise_utils.c
  src/tests/test_mls_query.c
  src/tests/test_code_extraction.c
  src/tests/test_location_decode.c
)

set(includes
  include
  include/utils
)

idf_component_register(
  SRCS ${srcs}
  INCLUDE_DIRS ${includes}
)

target_compile_options(${COMPONENT_LIB} PRIVATE
  -Wall
  -Wextra
  -Werror=double-promotion
  -Ofast
)
