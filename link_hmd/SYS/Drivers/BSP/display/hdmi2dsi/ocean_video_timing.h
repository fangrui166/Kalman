#ifndef __OCEAN_VIDEO_TIMING_H__
#define __OCEAN_VIDEO_TIMING_H__

#define OCEAN_H2D_MAGIC  (0x68326473)
#define OCEAN_DP_SOURCE_FPS  (60.026494)

#if COLOR_BAR_TEST
static const reg_item ocean_colorbar_phase1_tbl[] = {
    #include "ColorBarPhase1_ocean.tbl"
};
static const reg_item ocean_colorbar_phase2_tbl[] = {
    #include "ColorBarPhase2_ocean.tbl"
};
#endif

static const reg_item ocean_startup_phase1_tbl[] = {
    #include "StartupPhase1_ocean.tbl"
};

static const reg_item ocean_startup_phase2_tbl[] = {
    #include "StartupPhase2_ocean.tbl"
};

static const h2d_tbl_t ocean_h2d_tbl = {
    OCEAN_H2D_MAGIC,
    OCEAN_DP_SOURCE_FPS,
    sizeof(ocean_startup_phase1_tbl)/sizeof(ocean_startup_phase1_tbl[0]),
    ocean_startup_phase1_tbl,
    sizeof(ocean_startup_phase2_tbl)/sizeof(ocean_startup_phase2_tbl[0]),
    ocean_startup_phase2_tbl,
#if COLOR_BAR_TEST
    sizeof(ocean_colorbar_phase1_tbl)/sizeof(ocean_colorbar_phase1_tbl[0]),
    ocean_colorbar_phase1_tbl,
    sizeof(ocean_colorbar_phase2_tbl)/sizeof(ocean_colorbar_phase2_tbl[0]),
    ocean_colorbar_phase2_tbl
#else
    0,
    0,
    0,
    0
#endif
};

#endif //__OCEAN_VIDEO_TIME_H__
