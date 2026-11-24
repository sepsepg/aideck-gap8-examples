/*
 * streamer.c
 * Elia Cereda <elia.cereda@idsia.ch>
 *
 * Copyright (C) 2022-2025 IDSIA, USI-SUPSI
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
 * 
 * This software is based on the following publication:
 *    E. Cereda, A. Giusti, D. Palossi. "NanoCockpit: Performance-optimized 
 *    Application Framework for AI-based Autonomous Nanorobotics"
 * We kindly ask for a citation if you use in academic work.
 */

#include "streamer.h"

#include "config.h"
#include "camera.h"
#include "cpx/cpx.h"
#include "crc32.h"
#include "debug.h"
#include "time.h"
#include "trace.h"
#include "utils.h"

#include <pmsis.h>

#ifdef STREAMER_VERBOSE
    #define STREAMER_VERBOSE_PRINT(...) CO_PRINT(__VA_ARGS__)
#else
    #define STREAMER_VERBOSE_PRINT(...) 
#endif

typedef enum {
    STREAMER_CMD_BUFFER_BEGIN           = 0x10,
    STREAMER_CMD_BUFFER_DATA            = 0x11,
} __attribute__((packed)) streamer_cmd_e;

typedef struct streamer_begin_s {
    streamer_type_e type;
    uint32_t size;
    uint32_t checksum;
    uint8_t _padding[2];
} __attribute__((packed)) streamer_begin_t;

typedef struct streamer_data_s {
    uint8_t _padding[3];
} __attribute__((packed)) streamer_data_t;

typedef struct streamer_packet_s {
    streamer_cmd_e command;
    union {
        streamer_begin_t begin;
        streamer_data_t data;
    };
} __attribute__((packed)) streamer_packet_t;

static size_t streamer_packet_head_length(streamer_packet_t *packet) {
    size_t size = sizeof(packet->command);

    switch (packet->command) {
    case STREAMER_CMD_BUFFER_BEGIN:
        size += sizeof(packet->begin);
        break;

    case STREAMER_CMD_BUFFER_DATA:
        size += sizeof(packet->data);
        break;

    default:
        CO_ASSERTION_FAILURE("Unknown streamer_cmd_e %d", packet->command);
    }

    return size;
}

static streamer_packet_t *streamer_packet_init(cpx_send_req_t *cpx_req, streamer_cmd_e command) {
    streamer_packet_t *packet = (streamer_packet_t *)cpx_req->payload;
    packet->command = command;
    
    // First reset tail_length to zero, then set the head_length for the new packet.
    // This avoids asserting because the combined lengths exceed CPX_MTU
    cpx_send_req_set_tail(cpx_req, NULL, 0);
    cpx_send_req_set_head_length(cpx_req, streamer_packet_head_length(packet));
    return packet;
}

CO_FN_DECLARE(streamer_send_task);
CO_FN_DECLARE(streamer_cpx_callback);
#ifdef STREAMER_JPEG
CO_FN_DECLARE(streamer_jpeg_task);

// Upper bounds for the constant JPEG header and footer (actual: ~330 B and 2 B)
#define STREAMER_JPEG_HEADER_MAX    (1024)
#define STREAMER_JPEG_FOOTER_MAX    (16)
#endif
static void streamer_buffer_begin_received(streamer_t *streamer, streamer_packet_t *packet, uint16_t payload_length);
static void streamer_buffer_data_received(streamer_t *streamer, streamer_packet_t *packet, uint16_t payload_length);

void streamer_buffer_init(streamer_buffer_t *buffer, void *storage, size_t storage_capacity) {
    buffer->storage = storage;
    buffer->storage_capacity = storage_capacity;
    buffer->size = 0;
    buffer->received_size = 0;
    buffer->done_task = NULL;
}

void streamer_init(streamer_t *streamer, camera_t *camera, cpx_t *cpx) {
    streamer->camera = camera;

    streamer->cpx = cpx;
    streamer->cpx_req = cpx_send_req_alloc(sizeof(streamer_packet_t));
    streamer->cpx_req->header = CPX_HEADER_INIT(CPX_T_WIFI_HOST, CPX_F_STREAMER);

    streamer->buffer_rx = NULL;
    cpx_register_rx_callback(cpx, CPX_F_STREAMER, streamer_cpx_callback, (void *)streamer);

#if defined(STREAMER_DISABLE)
    VERBOSE_PRINT("Streamer init:\t\t\tDisabled\n");
#elif defined(__PLATFORM_GVSOC__)
    // GVSOC does not support SPIM, do nothing for now
    VERBOSE_PRINT("Streamer init:\t\t\tGVSOC\n");
#else
    VERBOSE_PRINT("Streamer init:\t\t\tOK\n");
#endif
}

void streamer_alloc_frames(streamer_t *streamer, camera_t *camera) {
    size_t buffer_size = camera_get_buffer_size(camera);
    size_t payload_size = sizeof(streamer_payload_t) + buffer_size;

    uint8_t *camera_buffers[CAMERA_BUFFERS];
    for (int i = 0; i < CAMERA_BUFFERS; i++) {
        streamer_payload_t *payload = pi_l2_malloc(payload_size);
        
        VERBOSE_PRINT(
            "Streamer buffer:\t\t%s, %dB @ L2, %p\n",
            payload ? "OK" : "Failed",
            payload_size, payload
        );

        if (payload == NULL) {
            CO_ASSERTION_FAILURE("Streamer buffer allocation failed.\n");
        }

        streamer->frames[i].streamer = streamer;
        streamer->frames[i].payload = payload;
        streamer->frames[i].payload_size = payload_size;
        
        camera_buffers[i] = payload->buffer;
    }

    camera_init_frames_external(camera, CAMERA_BUFFERS, camera_buffers, buffer_size);

#ifdef STREAMER_JPEG
    // Two output buffers: a frame is encoded into one while the previous frame is sent
    // from the other. The encoder checks the output size of every restart segment and
    // fails (the frame is skipped) instead of overflowing.
    size_t jpeg_payload_size = sizeof(streamer_payload_t) + STREAMER_JPEG_HEADER_MAX
                             + STREAMER_JPEG_BUFFER_SIZE + STREAMER_JPEG_FOOTER_MAX;

    for (int i = 0; i < STREAMER_JPEG_BUFFERS; i++) {
        streamer_payload_t *jpeg_payload = pi_l2_malloc(jpeg_payload_size);

        VERBOSE_PRINT(
            "Streamer JPEG buffer #%d:\t%s, %dB @ L2, %p\n",
            i, jpeg_payload ? "OK" : "Failed",
            jpeg_payload_size, jpeg_payload
        );

        if (jpeg_payload == NULL) {
            CO_ASSERTION_FAILURE("Streamer JPEG buffer allocation failed.\n");
        }

        streamer->jpeg_frames[i].streamer = streamer;
        streamer->jpeg_frames[i].payload = jpeg_payload;
        streamer->jpeg_frames[i].payload_size = jpeg_payload_size;
        streamer->jpeg_frames[i].data_size = 0;
    }
    streamer->jpeg_next = 0;
    streamer->jpeg_send_started = false;

    struct jpeg_encoder_conf jpeg_conf;
    jpeg_encoder_conf_init(&jpeg_conf);
    jpeg_conf.width = CAMERA_CROP_WIDTH;
    jpeg_conf.height = CAMERA_CROP_HEIGHT;
    jpeg_conf.flags = JPEG_ENCODER_FLAGS_CLUSTER_OFFLOAD;

    // jpeg_encoder_start opens (powers on) the cluster and loads the tables in its L1
    if (jpeg_encoder_open(&streamer->jpeg, &jpeg_conf) || jpeg_encoder_start(&streamer->jpeg)) {
        CO_ASSERTION_FAILURE("Streamer JPEG encoder init failed.\n");
    }

    // The JPEG header (tables, image size) is the same for every frame: write it once into
    // each buffer right after the metadata, the entropy data of each frame goes after it.
    pi_buffer_t header;
    uint32_t header_size = 0;
    pi_buffer_init(&header, PI_BUFFER_TYPE_L2, streamer->jpeg_frames[0].payload->buffer);
    header.size = STREAMER_JPEG_HEADER_MAX;
    if (jpeg_encoder_header_rst(&streamer->jpeg, &header, &header_size)
            || header_size == 0 || header_size > STREAMER_JPEG_HEADER_MAX) {
        CO_ASSERTION_FAILURE("Streamer JPEG header size %d invalid.\n", header_size);
    }
    for (int i = 1; i < STREAMER_JPEG_BUFFERS; i++) {
        memcpy(streamer->jpeg_frames[i].payload->buffer, streamer->jpeg_frames[0].payload->buffer,
               header_size);
    }
    streamer->jpeg_header_size = header_size;

    VERBOSE_PRINT("Streamer JPEG init:\t\tOK, %dx%d, q%d, %d restart segments, header %dB\n",
                  CAMERA_CROP_WIDTH, CAMERA_CROP_HEIGHT, STREAMER_JPEG_QUALITY,
                  JPEG_RST_SEGMENTS, header_size);
#endif
}

static size_t streamer_frame_get_size(streamer_frame_t *frame) {
    // Note: this function returns the size of the frame to be transmitted,
    // while frame->payload_size represents the capacity of the buffer.
    // The difference is due to acquiring camera images at a certain resolution
    // and then cropping (raw), or to the variable size of JPEG frames.
    return sizeof(streamer_payload_t) + frame->data_size;
}

static uint32_t streamer_compute_checksum(const void* buffer, size_t size) {
    uint32_t checksum = crc32CalculateBuffer(buffer, size);

    if (checksum == 0) {
        // If the computed checksum is zero, it is transmitted as all ones. An all zero
        // transmitted checksum value means that the transmitter generated no checksum.
        checksum = UINT32_MAX;
    }

    return checksum;
}

void streamer_send_frame_async(
    streamer_t *streamer,
    frame_t *camera_frame,
    state_msg_t *state, uint32_t state_timestamp,
    tof_msg_t *tof, uint32_t tof_timestamp,
    inference_stamped_msg_t *inference,
    pi_task_t *done_task
) {
#if defined(STREAMER_DISABLE) || defined(__PLATFORM_GVSOC__)
    // GVSOC does not support SPIM, do nothing for now
    // TODO: provide a custom CPX transport over a pipe so that the streamer can be used from GVSOC
    if (done_task) {
        pi_task_push(done_task);
    }
    return;
#endif

    camera_t *camera = streamer->camera;
    int buffer_id = camera_get_buffer_id(camera, camera_frame);
    streamer_frame_t *frame = &streamer->frames[buffer_id];
    frame->payload->metadata = (streamer_metadata_t){
        .metadata_version = STREAMER_METADATA_VERSION,

        .frame_height = CAMERA_CROP_HEIGHT, // TODO: get from camera frame
        .frame_width = CAMERA_CROP_WIDTH,
        .frame_bpp = CAMERA_CROP_BPP,
        .frame_format = STREAMER_FORMAT_GRAY_8,
        .frame_id = camera_frame->frame_id,
        .frame_timestamp = camera_frame->frame_timestamp,

        .state = *state,
        .state_timestamp = state_timestamp,

        .tof = *tof,
        .tof_timestamp = tof_timestamp,

        .reply_frame_timestamp = streamer->reply_frame_timestamp,
        .reply_recv_timestamp = streamer->reply_recv_timestamp,

        .inference = *inference
    };
    frame->data_size = CAMERA_CROP_HEIGHT * CAMERA_CROP_WIDTH * CAMERA_CROP_BPP;

#ifdef STREAMER_JPEG
    co_fn_push_start(&streamer->jpeg_ctx, streamer_jpeg_task, (void *)frame, done_task);
#else
    co_fn_push_start(&frame->send_ctx, streamer_send_task, (void *)frame, done_task);
#endif
}

#ifdef STREAMER_JPEG
// Encode the (cropped) camera frame to JPEG on the cluster into the next output buffer,
// start its transmission and return: done_task fires as soon as the camera frame's buffer
// is no longer needed, so the caller can encode the next frame while this one is sent.
//
// Transmissions stay one at a time and in order: before starting one, wait for the previous
// one (it ran during this encode, so normally it has already finished; if Wi-Fi is slow,
// this wait is what holds the camera back). With two output buffers this also guarantees
// that the buffer being encoded into is not being sent: the only transmission that can
// still be running is the previous frame's, from the other buffer.
//
// Frames whose JPEG would not fit the output buffer are skipped (counted in the stats).
CO_FN_BEGIN(streamer_jpeg_task, streamer_frame_t *, frame)
{
#ifdef STREAMER_JPEG_STATS
    static uint32_t stat_t0, stat_frames, stat_skipped, stat_bytes, stat_enc_us, stat_enc_max;
    static uint32_t stat_wait_us, stat_wait_max;
#endif
    static uint32_t t_enc, t_wait;
    static int32_t status;
    // Only one frame at a time is encoded (the caller waits for done_task before passing
    // the next one), so static variables can be used. Static (L2) storage also keeps them
    // readable by the cluster.
    static streamer_t *streamer;
    static streamer_frame_t *jpeg_frame;
    static pi_buffer_t image;
    static pi_buffer_t bitstream;
    static pi_buffer_t footer;
    static uint32_t body_size;
    static uint32_t footer_size;
    static uint8_t *data;

    streamer = frame->streamer;
    jpeg_frame = &streamer->jpeg_frames[streamer->jpeg_next];
    data = jpeg_frame->payload->buffer;

    trace_set(TRACE_STREAMER_SEND, true);

    pi_buffer_init(&image, PI_BUFFER_TYPE_L2, frame->payload->buffer);
    image.size = CAMERA_CROP_WIDTH * CAMERA_CROP_HEIGHT;
    image.width = CAMERA_CROP_WIDTH;
    image.height = CAMERA_CROP_HEIGHT;

    pi_buffer_init(&bitstream, PI_BUFFER_TYPE_L2, data + streamer->jpeg_header_size);
    bitstream.size = STREAMER_JPEG_BUFFER_SIZE;

    t_enc = time_get_us();
    jpeg_encoder_process_rst_async(&streamer->jpeg, &image, &bitstream, co_event_init(&streamer->jpeg_done));
    CO_WAIT(&streamer->jpeg_done);
    t_enc = time_get_us() - t_enc;

    body_size = 0;
    status = jpeg_encoder_process_rst_status(&body_size);

    if (status) {
        // A band had too much detail for its share of the output buffer: skip the frame
        // (the buffer stays free for the next one)
        trace_set(TRACE_STREAMER_SEND, false);
#ifdef STREAMER_JPEG_STATS
        stat_skipped++;
        stat_frames++;
#endif
        break;
    }

    pi_buffer_init(&footer, PI_BUFFER_TYPE_L2, data + streamer->jpeg_header_size + body_size);
    footer.size = STREAMER_JPEG_FOOTER_MAX;
    footer_size = 0;
    jpeg_encoder_footer(&streamer->jpeg, &footer, &footer_size);

    jpeg_frame->payload->metadata = frame->payload->metadata;
    jpeg_frame->payload->metadata.frame_format = STREAMER_FORMAT_JPEG;
    jpeg_frame->data_size = streamer->jpeg_header_size + body_size + footer_size;

    trace_set(TRACE_STREAMER_SEND, false);

    // One transmission at a time (streamer_send_task and the CPX request are shared)
    t_wait = time_get_us();
    if (streamer->jpeg_send_started) {
        while (!co_event_is_done(&streamer->jpeg_sent)) {
            CO_WAIT(&streamer->jpeg_sent);
        }
    }
    t_wait = time_get_us() - t_wait;

#ifdef STREAMER_JPEG_STATS
    // Printed over the JTAG console only: never enable it in a flashed image (with no debugger
    // attached, the console output can block)
    stat_frames++;
    stat_enc_us += t_enc;
    if (t_enc > stat_enc_max) stat_enc_max = t_enc;
    stat_wait_us += t_wait;
    if (t_wait > stat_wait_max) stat_wait_max = t_wait;
    stat_bytes += body_size;
    if (stat_t0 == 0) stat_t0 = time_get_us();
    if (stat_frames >= 128) {
        uint32_t dt = time_get_us() - stat_t0;
        VERBOSE_PRINT("JPEG: %d frames in %d ms, encode avg %d us max %d us, "
                      "wait for previous send avg %d us max %d us, avg %d B, skipped %d\n",
                      stat_frames, dt / 1000, stat_enc_us / stat_frames, stat_enc_max,
                      stat_wait_us / stat_frames, stat_wait_max,
                      stat_bytes / MAX(1, stat_frames - stat_skipped), stat_skipped);
        stat_t0 = time_get_us();
        stat_frames = stat_skipped = stat_bytes = stat_enc_us = stat_enc_max = 0;
        stat_wait_us = stat_wait_max = 0;
    }
#endif

    streamer->jpeg_send_started = true;
    streamer->jpeg_next = (streamer->jpeg_next + 1) % STREAMER_JPEG_BUFFERS;
    co_fn_push_start(&jpeg_frame->send_ctx, streamer_send_task, (void *)jpeg_frame,
                     co_event_init(&streamer->jpeg_sent));

    // Return without waiting for the transmission: the camera frame is no longer needed
}
CO_FN_END()
#endif

CO_FN_BEGIN(streamer_send_task, streamer_frame_t *, frame)
{
    // Only one send at a time must be inside this critical section, so static variables can be used
    static streamer_t *streamer;
    static ssize_t frame_size;
    static streamer_packet_t *packet;

    trace_set(TRACE_STREAMER_SEND, true);

    streamer = frame->streamer;
    frame_size = streamer_frame_get_size(frame);

#ifdef STREAMER_SEND_CHECKSUM
    uint32_t checksum = streamer_compute_checksum(frame->payload, frame_size);
#else
    uint32_t checksum = 0;
#endif

    static uint8_t *packet_payload;
    static ssize_t remaining_length;
    static uint16_t packet_length;

    packet_payload = (uint8_t *)frame->payload;
    remaining_length = frame_size;

    while (remaining_length > 0) {
        if (remaining_length == frame_size) {
            packet = streamer_packet_init(streamer->cpx_req, STREAMER_CMD_BUFFER_BEGIN);
            packet->begin = (streamer_begin_t){
                .type = STREAMER_TYPE_IMAGE,
                .size = frame_size,
                .checksum = checksum
            };
        } else {
            packet = streamer_packet_init(streamer->cpx_req, STREAMER_CMD_BUFFER_DATA);
        }

        packet_length = MIN(remaining_length, cpx_send_req_max_tail_length(streamer->cpx_req));
        if (packet_length == remaining_length && (packet_length % 4) != 0) {
            packet_length += 4 - (packet_length % 4);
        }

        cpx_send_req_set_tail(streamer->cpx_req, packet_payload, packet_length);
        cpx_send_async(streamer->cpx, streamer->cpx_req, co_event_init(&streamer->cpx_done));
        CO_WAIT(&streamer->cpx_done);

        packet_payload += packet_length;
        remaining_length -= packet_length;
    }

    trace_set(TRACE_STREAMER_SEND, false);
}
CO_FN_END()

void streamer_receive_buffer_async(streamer_t *streamer, streamer_buffer_t *buffer, pi_task_t *done_task) {
    if (done_task == NULL) {
        CO_ASSERTION_FAILURE("Must specify a callback task\n");
    }

    if (buffer->done_task != NULL) {
        CO_ASSERTION_FAILURE("Buffer is already in use\n");
    }
    buffer->done_task = done_task;
    
    if (streamer->buffer_rx != NULL) {
        CO_ASSERTION_FAILURE("Streamer is already waiting to receive another buffer\n");
    }
    streamer->buffer_rx = buffer;
}

void streamer_cancel_receive(streamer_t *streamer, streamer_buffer_t *buffer) {
    if (streamer->buffer_rx == NULL && buffer->done_task == NULL) {
        // Buffer already received, nothing to do
        return;
    }

    if (streamer->buffer_rx != buffer) {
        CO_ASSERTION_FAILURE("Streamer is waiting to receive (%p), not the buffer being canceled (%p)\n", streamer->buffer_rx, buffer);
    }
    
    buffer->size = 0;
    buffer->received_size = 0;
    pi_task_push(buffer->done_task);
    buffer->done_task = NULL;
    streamer->buffer_rx = NULL;
}

CO_FN_BEGIN(streamer_cpx_callback, cpx_receive_req_t *, req)
{
    trace_set(TRACE_STREAMER_RECEIVE, false);

    streamer_t *streamer = (streamer_t *)req->receiver_args;
    streamer_packet_t *packet = (streamer_packet_t *)req->payload;

    if (packet->command == STREAMER_CMD_BUFFER_BEGIN) {
        streamer_buffer_begin_received(streamer, packet, req->payload_length);
    } else if (packet->command == STREAMER_CMD_BUFFER_DATA) {
        streamer_buffer_data_received(streamer, packet, req->payload_length);
    } else {
        VERBOSE_PRINT("Unknown streamer packet (command: 0x%02x)\n", packet->command);
        break;
    }
}
CO_FN_END();

static void streamer_buffer_begin_received(streamer_t *streamer, streamer_packet_t *packet, uint16_t payload_length) {
    streamer_buffer_t *buffer = streamer->buffer_rx;
    if (buffer == NULL) {
        STREAMER_VERBOSE_PRINT("No receiver waiting for buffer, dropping segment\n");
        return;
    }

    uint32_t buffer_size = packet->begin.size;
    if (buffer_size > buffer->storage_capacity) {
        VERBOSE_PRINT("Buffer (%d bytes) too big for receiver storage (%d bytes), dropping segment\n", buffer_size, buffer->storage_capacity);
        return;
    }

    buffer->type = packet->begin.type;
    buffer->size = buffer_size;
    buffer->received_size = 0;
    buffer->checksum = packet->begin.checksum;

    // The payload of BUFFER_BEGIN contains the first segment of the transmitted buffer
    streamer_buffer_data_received(streamer, packet, payload_length);

    trace_set(TRACE_STREAMER_RECEIVE, true);
}

static void streamer_buffer_data_received(streamer_t *streamer, streamer_packet_t *packet, uint16_t payload_length) {
    streamer_buffer_t *buffer = streamer->buffer_rx;
    if (buffer == NULL) {
        STREAMER_VERBOSE_PRINT("No receiver waiting for buffer, dropping segment\n");
        return;
    }

    if (buffer->size == 0) {
        STREAMER_VERBOSE_PRINT("Was not expecting data, dropping segment\n");
        return;
    }

    size_t header_length = streamer_packet_head_length(packet);
    void *segment = (void *)packet + header_length;
    size_t  segment_length = payload_length - header_length;

    size_t remaining_length = buffer->size - buffer->received_size;
    if (segment_length > remaining_length) {
        buffer->size = 0;
        buffer->received_size = 0;
        buffer->checksum = 0;
        STREAMER_VERBOSE_PRINT("Was expecting up to %d bytes but received %d bytes, dropping buffer\n", remaining_length, segment_length);
        return;
    }

    trace_set(TRACE_STREAMER_RECEIVE, true);

    memcpy(buffer->storage + buffer->received_size, segment, segment_length);
    buffer->received_size += segment_length;

    if (buffer->received_size == buffer->size) {
#ifdef STREAMER_RECEIVE_CHECKSUM
        if (buffer->checksum != 0) {
            uint32_t received_checksum = streamer_compute_checksum(buffer->storage, buffer->size);
            if (received_checksum != buffer->checksum) {
                buffer->size = 0;
                buffer->received_size = 0;
                buffer->checksum = 0;
                VERBOSE_PRINT("Received buffer is corrupted (checksum %u, expected %u)\n", received_checksum, buffer->checksum);
                return;
            }
        }
#endif

        pi_task_push(buffer->done_task);
        buffer->done_task = NULL;
        streamer->buffer_rx = NULL;

        trace_set(TRACE_STREAMER_RECEIVE, false);
    }
}

void streamer_stats_frame_completed(streamer_t *streamer, streamer_stats_t *stats) {
    streamer->reply_frame_timestamp = stats->reply_frame_timestamp;
    streamer->reply_recv_timestamp = time_get_us();
}
