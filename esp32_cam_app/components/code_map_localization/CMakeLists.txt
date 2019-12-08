set(COMPONENT_SRCS
  src/code_extraction.c
  src/location_decode.c
  src/mls_index.c
  src/mls_query.c
  src/utils/bitwise_utils.c
  src/utils/image_utils.c

#for unit tests
  src/utils/test_utils.c
  src/tests/tests.c
  src/tests/test_bitwise_utils.c
  src/tests/test_full_chain.c
  src/tests/test_location_decode.c
  src/tests/test_math_utils.c
  src/tests/test_mls_query.c
  )

set(COMPONENT_ADD_INCLUDEDIRS
  include
  include/utils
  )

register_component()
