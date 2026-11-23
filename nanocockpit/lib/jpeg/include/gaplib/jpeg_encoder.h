/*
 * Copyright (C) 2019 GreenWaves Technologies
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __KERNELS__JPEG_ENCODER_H__
#define __KERNELS__JPEG_ENCODER_H__

#include <bsp/buffer.h>

typedef enum {
  JPEG_ENCODER_FLAGS_CLUSTER_OFFLOAD = 1 << 0
} jpeg_encoder_flags_e;


struct jpeg_encoder_conf
{
  unsigned int width;
  unsigned int height;
  jpeg_encoder_flags_e flags;
};


typedef struct jpeg_encoder_s
{
  unsigned int current_du_x;
  unsigned int current_du_y;
  int pending_bits;
  short int prev_du;
  unsigned char pending_bitstream_char;
  unsigned int width;
  unsigned int height;
  jpeg_encoder_flags_e flags;
  struct pi_device cluster_dev;
  struct pi_cluster_task cluster_task;
  struct pi_task cluster_end_task;
  void *cl_blocks;
  pi_buffer_t *cl_image;
  uint8_t *bitstream;
  int bitstream_size;
  pi_task_t *end_task;
  void *l1_constants;
} jpeg_encoder_t;


void jpeg_encoder_conf_init(struct jpeg_encoder_conf *conf);


int32_t jpeg_encoder_open(jpeg_encoder_t *enc, struct jpeg_encoder_conf *conf);

int32_t jpeg_encoder_start(jpeg_encoder_t *enc);

int32_t jpeg_encoder_header(jpeg_encoder_t *enc, pi_buffer_t *encoded_bitstream, uint32_t *encoded_size);

int32_t jpeg_encoder_footer(jpeg_encoder_t *enc, pi_buffer_t *encoded_bitstream, uint32_t *encoded_size);

void jpeg_encoder_stop(jpeg_encoder_t *enc);

void jpeg_encoder_close(jpeg_encoder_t *enc);

int32_t jpeg_encoder_process(jpeg_encoder_t *enc, pi_buffer_t *image, pi_buffer_t *encoded_bitstream, uint32_t *encoded_size);

void jpeg_encoder_process_async(jpeg_encoder_t *enc, pi_buffer_t *image, pi_buffer_t *encoded_bitstream, pi_task_t *task);

int32_t jpeg_encoder_process_status(uint32_t *encoded_size, pi_task_t *task);

/*
 * Parallel encoding with restart intervals (crash_monitor, 2026-11-23).
 *
 * The image is split into JPEG_RST_SEGMENTS restart intervals that the cluster cores encode
 * independently (each with its own DC predictor, using the same per-block code as the FC path),
 * then joined with RSTn markers. Unlike __jpeg_encoder_process_cl, no state is shared between
 * cores, and each segment stops (the frame is reported as failed) before it could overflow its
 * share of the output buffer. Needs jpeg_encoder_start() (cluster open, tables in L1) and a
 * header from jpeg_encoder_header_rst() (it carries the DRI marker).
 */
#define JPEG_RST_SEGMENTS 8

// Like jpeg_encoder_header, plus a DRI (define restart interval) marker before SOS
int32_t jpeg_encoder_header_rst(jpeg_encoder_t *enc, pi_buffer_t *encoded_bitstream, uint32_t *encoded_size);

// Encode the entropy-coded data (segments + RST markers) of a whole image into encoded_bitstream
// on the cluster; task is pushed when done. Then read the result with jpeg_encoder_process_rst_status.
void jpeg_encoder_process_rst_async(jpeg_encoder_t *enc, pi_buffer_t *image, pi_buffer_t *encoded_bitstream, pi_task_t *task);

// 0 and the encoded size, or -1 (size 0) if a segment did not fit in its share of the buffer
int32_t jpeg_encoder_process_rst_status(uint32_t *encoded_size);

#endif