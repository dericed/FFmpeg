/*
 * Copyright (c) 2017 Paul B Mahol
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/**
 * @file
 * CC line scanner
 */

#include <string.h>

#include "libavutil/internal.h"
#include "libavutil/opt.h"
#include "libavutil/pixdesc.h"
#include "libavutil/timestamp.h"

#include "avfilter.h"
#include "formats.h"
#include "internal.h"
#include "video.h"

#define FALL 0
#define RISE 1

typedef struct CCLineContext {
    const AVClass *class;
    int line;
    int min_range;
    int max_peak_diff;
    int max_start_diff;
    float mpd, msd, mac, spw, bhd;
    int chp;
} CCLineContext;

#define OFFSET(x) offsetof(CCLineContext, x)
#define FLAGS AV_OPT_FLAG_VIDEO_PARAM|AV_OPT_FLAG_FILTERING_PARAM

static const AVOption ccline_options[] = {
    { "line",  "set which line to scan for codes",                                 OFFSET(line), AV_OPT_TYPE_INT,   {.i64=20},  INT_MIN, INT_MAX, FLAGS },
    { "mac",   "set minimal acceptable amplitude change for sync codes detection", OFFSET(mac),  AV_OPT_TYPE_FLOAT, {.dbl=.20},   0.001,       1, FLAGS },
    { "spw",   "set ratio of width reserved for sync code detection",              OFFSET(spw),  AV_OPT_TYPE_FLOAT, {.dbl=.27},    0.01,    0.70, FLAGS },
    { "mpd",   "set max peaks height difference for sync code detection",          OFFSET(mpd),  AV_OPT_TYPE_FLOAT, {.dbl=.08},       0,    0.50, FLAGS },
    { "msd",   "set first two max start code bits differences",                    OFFSET(msd),  AV_OPT_TYPE_FLOAT, {.dbl=.04},       0,    0.50, FLAGS },
    { "bhd",   "set min ratio of bits height compared to 3rd start code bit",      OFFSET(bhd),  AV_OPT_TYPE_FLOAT, {.dbl=.75},    0.01,       1, FLAGS },
    { "chp",   "check and apply parity bit",                                       OFFSET(chp),  AV_OPT_TYPE_BOOL,  {.i64= 0},        0,       1, FLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(ccline);

static int query_formats(AVFilterContext *ctx)
{
    static const enum AVPixelFormat pixel_fmts[] = {
        AV_PIX_FMT_GRAY8,
        AV_PIX_FMT_YUV410P, AV_PIX_FMT_YUV411P,
        AV_PIX_FMT_YUV420P, AV_PIX_FMT_YUV422P,
        AV_PIX_FMT_YUV440P, AV_PIX_FMT_YUV444P,
        AV_PIX_FMT_YUVJ420P, AV_PIX_FMT_YUVJ422P,
        AV_PIX_FMT_YUVJ440P, AV_PIX_FMT_YUVJ444P,
        AV_PIX_FMT_YUVJ411P,
        AV_PIX_FMT_NONE
    };
    AVFilterFormats *formats = ff_make_format_list(pixel_fmts);
    if (!formats)
        return AVERROR(ENOMEM);
    return ff_set_common_formats(ctx, formats);
}

static int config_input(AVFilterLink *inlink)
{
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(inlink->format);
    AVFilterContext *ctx = inlink->dst;
    CCLineContext *s = ctx->priv;
    int depth = desc->comp[0].depth;

    if (FFABS(s->line) >= inlink->h) {
        av_log(ctx, AV_LOG_WARNING, "line must be lower than video height, clipping\n");
        s->line = av_clip(s->line, -(inlink->h - 1), inlink->h - 1);
    }

    s->min_range = s->mac * ((1 << depth) - 1);
    s->max_peak_diff = s->mpd * ((1 << depth) - 1);
    s->max_start_diff = s->msd * ((1 << depth) - 1);

    return 0;
}

static void extract_line(AVFilterContext *ctx, AVFilterLink *inlink, AVFrame *in, int line)
{
    CCLineContext *s = ctx->priv;
    int max = 0, min = INT_MAX;
    int i, ch, range = 0;
    const uint8_t *src;
    uint8_t clock[8] = { 0 };
    const int sync_width = s->spw * in->width;
    int last = 0, peaks = 0, max_peak_diff = 0, dir = RISE;
    const int width_per_bit = (in->width - sync_width) / 19;
    uint8_t byte[2] = { 0 };
    int s1, s2, s3, parity;

    src = &in->data[0][line * in->linesize[0]];
    for (i = 0; i < in->width; i++) {
        max = FFMAX(max, src[i]);
        min = FFMIN(min, src[i]);
    }

    range = max - min;
    if (range < s->min_range)
        return;

    for (i = 0; i < sync_width; i += 2) {
        int Y = src[i];

        if (dir == RISE) {
            if (Y < last) {
                dir = FALL;
                if (Y >= s->min_range) {
                    clock[peaks] = Y;
                    peaks++;
                    if (peaks > 7)
                        break;
                }
            }
        } else if (dir == FALL) {
            if (Y > last) {
                dir = RISE;
            }
        }
        last = Y;
    }

    if (peaks != 7)
        return;

    for (i = 1; i < 7; i++)
        max_peak_diff = FFMAX(max_peak_diff, FFABS(clock[i] - clock[i-1]));

    if (max_peak_diff > s->max_peak_diff)
        return;

    s1 = src[sync_width + width_per_bit * 0 + width_per_bit / 2];
    s2 = src[sync_width + width_per_bit * 1 + width_per_bit / 2];
    s3 = src[sync_width + width_per_bit * 2 + width_per_bit / 2];

    if (FFABS(s1 - s2) > s->max_start_diff || s1 >= s3 || s2 >= s3)
        return;

    for (ch = 0; ch < 2; ch++) {
        for (parity = 0, i = 0; i < 8; i++) {
            int b = src[sync_width + width_per_bit * (i + 3 + 8 * ch) + width_per_bit / 2];

            if (b - s1 > (s3 - s1) * s->bhd) {
                b = 1;
                parity++;
            } else {
                b = 0;
            }
            byte[ch] |= b << i;
        }

        if (s->chp) {
            if (parity & 1) {
                byte[ch] &= 0x7F;
            } else {
                byte[ch] = 0;
            }
        }
    }

    {
        uint8_t key[128];
        uint8_t value[8];

        snprintf(key, sizeof(key), "lavfi.ccline.line.%d", line);
        snprintf(value, sizeof(value), "0x%02X%02X", byte[0], byte[1]);
        av_dict_set(avpriv_frame_get_metadatap(in), key, value, 0);
    }
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx  = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    CCLineContext *s = ctx->priv;

    if (s->line < 0) {
        int i;

        for (i = 0; i < -s->line; i++)
            extract_line(ctx, inlink, in, i);
    } else {
        extract_line(ctx, inlink, in, s->line);
    }

    return ff_filter_frame(outlink, in);
}

static const AVFilterPad avfilter_vf_ccline_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .filter_frame = filter_frame,
        .config_props = config_input,
    },
    { NULL }
};

static const AVFilterPad avfilter_vf_ccline_outputs[] = {
    {
        .name = "default",
        .type = AVMEDIA_TYPE_VIDEO,
    },
    { NULL }
};

AVFilter ff_vf_ccline = {
    .name          = "ccline",
    .description   = NULL_IF_CONFIG_SMALL("Extract Closed Captioning codes from input video into frame metadata."),
    .priv_size     = sizeof(CCLineContext),
    .priv_class    = &ccline_class,
    .query_formats = query_formats,
    .inputs        = avfilter_vf_ccline_inputs,
    .outputs       = avfilter_vf_ccline_outputs,
    .flags         = AVFILTER_FLAG_SUPPORT_TIMELINE_GENERIC,
};
