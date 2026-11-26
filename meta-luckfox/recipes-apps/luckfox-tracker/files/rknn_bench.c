#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include "rknn_api.h"

static double now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
}

int main(int argc, char **argv) {
    const char *model_path = (argc > 1) ? argv[1] : "/tmp/simo_lite_test.rknn";

    rknn_context ctx = 0;
    // On RV1106 librknnmrt, passing file path directly lets runtime map weights cleanly
    int ret = rknn_init(&ctx, (void *)model_path, 0, 0, NULL);
    if (ret < 0) {
        fprintf(stderr, "rknn_init failed: %d\n", ret);
        return 1;
    }

    rknn_input_output_num io_num;
    rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
    printf("Model loaded: inputs=%u outputs=%u\n", io_num.n_input, io_num.n_output);

    rknn_tensor_attr in_attr = { .index = 0 };
    ret = rknn_query(ctx, RKNN_QUERY_NATIVE_NHWC_INPUT_ATTR, &in_attr, sizeof(in_attr));
    if (ret < 0) ret = rknn_query(ctx, RKNN_QUERY_INPUT_ATTR, &in_attr, sizeof(in_attr));

    rknn_tensor_attr out_attr = { .index = 0 };
    ret = rknn_query(ctx, RKNN_QUERY_NATIVE_NHWC_OUTPUT_ATTR, &out_attr, sizeof(out_attr));
    if (ret < 0) ret = rknn_query(ctx, RKNN_QUERY_NATIVE_OUTPUT_ATTR, &out_attr, sizeof(out_attr));

    printf("IN : dims=[%u,%u,%u,%u] fmt=%d type=%d size=%u zp=%d scale=%f\n",
           in_attr.dims[0], in_attr.dims[1], in_attr.dims[2], in_attr.dims[3],
           in_attr.fmt, in_attr.type, in_attr.size_with_stride, in_attr.zp, in_attr.scale);
    printf("OUT: dims=[%u,%u,%u,%u] fmt=%d type=%d size=%u zp=%d scale=%f\n",
           out_attr.dims[0], out_attr.dims[1], out_attr.dims[2], out_attr.dims[3],
           out_attr.fmt, out_attr.type, out_attr.size_with_stride, out_attr.zp, out_attr.scale);

    uint32_t in_sz  = in_attr.size_with_stride  ? in_attr.size_with_stride  : in_attr.size;
    uint32_t out_sz = out_attr.size_with_stride ? out_attr.size_with_stride : out_attr.size;

    rknn_tensor_mem *in_mem  = rknn_create_mem(ctx, in_sz);
    rknn_tensor_mem *out_mem = rknn_create_mem(ctx, out_sz);
    if (!in_mem || !out_mem) {
        fprintf(stderr, "rknn_create_mem failed\n");
        return 1;
    }

    in_attr.pass_through = 0;
    if (rknn_set_io_mem(ctx, in_mem, &in_attr) < 0 ||
        rknn_set_io_mem(ctx, out_mem, &out_attr) < 0) {
        fprintf(stderr, "rknn_set_io_mem failed\n");
        return 1;
    }

    memset(in_mem->virt_addr, 17, in_sz);

    // Warmup
    rknn_mem_sync(ctx, in_mem, RKNN_MEMORY_SYNC_TO_DEVICE);
    ret = rknn_run(ctx, NULL);
    rknn_mem_sync(ctx, out_mem, RKNN_MEMORY_SYNC_FROM_DEVICE);
    if (ret < 0) {
        fprintf(stderr, "Warmup rknn_run failed: %d\n", ret);
        return 1;
    }

    double min_ms = 1e9, max_ms = 0, sum_ms = 0;
    int fails = 0;
    const int N = 200;
    for (int i = 0; i < N; i++) {
        double t0 = now_ms();
        rknn_mem_sync(ctx, in_mem, RKNN_MEMORY_SYNC_TO_DEVICE);
        ret = rknn_run(ctx, NULL);
        rknn_mem_sync(ctx, out_mem, RKNN_MEMORY_SYNC_FROM_DEVICE);
        double dt = now_ms() - t0;
        if (ret < 0) fails++;
        if (dt < min_ms) min_ms = dt;
        if (dt > max_ms) max_ms = dt;
        sum_ms += dt;
        usleep(10000); // ~60-100 Hz pacing while testing alongside vtx
    }

    printf("Bench (%d runs): avg=%.2f ms, min=%.2f ms, max=%.2f ms, fails=%d\n",
           N, sum_ms / N, min_ms, max_ms, fails);

    rknn_destroy_mem(ctx, in_mem);
    rknn_destroy_mem(ctx, out_mem);
    rknn_destroy(ctx);
    return 0;
}