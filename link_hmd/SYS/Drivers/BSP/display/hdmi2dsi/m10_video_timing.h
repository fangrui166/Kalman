#ifndef __M10_VIDEO_TIMING_H__
#define __M10_VIDEO_TIMING_H__

#define M10_H2D_MAGIC  (0x68326473)
#define M10_DP_SOURCE_FPS  (58.252312)

#if COLOR_BAR_TEST
static const reg_item m10_colorbar_phase1_tbl[] = {
    #include "ColorBarPhase1_m10.tbl"
};
static const reg_item m10_colorbar_phase2_tbl[] = {
    #include "ColorBarPhase2_m10.tbl"
};
#endif

static const reg_item m10_startup_phase1_tbl[] = {
    #include "StartupPhase1_m10.tbl"
};

static const reg_item m10_startup_phase2_tbl[] = {
    #include "StartupPhase2_m10.tbl"
};

static const h2d_tbl_t m10_h2d_tbl = {
    M10_H2D_MAGIC,
    M10_DP_SOURCE_FPS,
    sizeof(m10_startup_phase1_tbl)/sizeof(m10_startup_phase1_tbl[0]),
    m10_startup_phase1_tbl,
    sizeof(m10_startup_phase2_tbl)/sizeof(m10_startup_phase2_tbl[0]),
    m10_startup_phase2_tbl,
#if COLOR_BAR_TEST
    sizeof(m10_colorbar_phase1_tbl)/sizeof(m10_colorbar_phase1_tbl[0]),
    m10_colorbar_phase1_tbl,
    sizeof(m10_colorbar_phase2_tbl)/sizeof(m10_colorbar_phase2_tbl[0]),
    m10_colorbar_phase2_tbl
#else
    0,
    0,
    0,
    0
#endif
};

#endif //__M10_VIDEO_TIME_H__
