#include "image_filter.h"

void imf_convolution(ImageMatrix* dst, const ImageMatrix src, const ImageMatrix kernel);

void imf_edge_filter(ImageMatrix* dst, const ImageMatrix src);
void imf_hough_line_transform(ImageMatrix dst, const ImageMatrix src);

void imf_convert_uint8_to_int16(ImageMatrix mat);
void imf_convert_int16_to_uint8(ImageMatrix mat);

int32_t imf_count_negative(ImageMatrix mat);
