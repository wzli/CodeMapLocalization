set(COMPONENT_SRCS
  src/code_extraction.c
  src/location_decode.c
  src/mls_index.c
  src/mls_query.c
  src/utils/bitwise_utils.c
  src/utils/image_utils.c
  )

set(COMPONENT_ADD_INCLUDEDIRS
  include
  include/utils
  )

register_component()
