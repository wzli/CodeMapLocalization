#include "image_utils.h"

void img_convolution(ImageMatrix* dst, const ImageMatrix src, const ImageMatrix kernel);

void img_edge_filter(ImageMatrix* dst, const ImageMatrix src);
void img_hough_line_transform(ImageMatrix dst, const ImageMatrix src);

void img_convert_uint8_to_int16(ImageMatrix mat);
void img_convert_int16_to_uint8(ImageMatrix mat);

int32_t img_count_negative(ImageMatrix mat);
