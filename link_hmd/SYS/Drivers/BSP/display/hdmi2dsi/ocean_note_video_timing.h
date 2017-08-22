#ifndef __OCEAN_NOTE_VIDEO_TIMING_H__
#define __OCEAN_NOTE_VIDEO_TIMING_H__

#define OCEAN_NOTE_H2D_MAGIC  (0x68326473)
#define OCEAN_NOTE_DP_SOURCE_FPS  (57.069524)

#if COLOR_BAR_TEST
static const reg_item ocean_note_colorbar_phase1_tbl[] = {
    #include "ColorBarPhase1_ocean_note.tbl"
};
static const reg_item ocean_note_colorbar_phase2_tbl[] = {
    #include "ColorBarPhase2_ocean_note.tbl"
};
#endif

static const reg_item ocean_note_startup_phase1_tbl[] = {
    #include "StartupPhase1_ocean_note.tbl"
};

static const reg_item ocean_note_startup_phase2_tbl[] = {
    #include "StartupPhase2_ocean_note.tbl"
};

static const h2d_tbl_t ocean_note_h2d_tbl = {
    OCEAN_NOTE_H2D_MAGIC,
    OCEAN_NOTE_DP_SOURCE_FPS,
    sizeof(ocean_note_startup_phase1_tbl)/sizeof(ocean_note_startup_phase1_tbl[0]),
    ocean_note_startup_phase1_tbl,
    sizeof(ocean_note_startup_phase2_tbl)/sizeof(ocean_note_startup_phase2_tbl[0]),
    ocean_note_startup_phase2_tbl,
#if COLOR_BAR_TEST
    sizeof(ocean_note_colorbar_phase1_tbl)/sizeof(ocean_note_colorbar_phase1_tbl[0]),
    ocean_note_colorbar_phase1_tbl,
    sizeof(ocean_note_colorbar_phase2_tbl)/sizeof(ocean_note_colorbar_phase2_tbl[0]),
    ocean_note_colorbar_phase2_tbl
#else
    0,
    0,
    0,
    0
#endif
};

#endif //__OCEAN_NOTE_VIDEO_TIME_H__
