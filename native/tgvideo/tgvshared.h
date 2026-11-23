/*
 * Layout of the global chunk "tgvideo_frames" shared by tgvideo.exe (writer)
 * and tgvdisp.pyd (reader). Plain 32 bit words, no pointers.
 *
 * Seqlock: the writer makes the seq word ODD before it touches a buffer and
 * EVEN again (and bumps the frame counter) afterwards. A reader accepts a
 * picture only if seq was even and unchanged before and after reading.
 */
#ifndef TGVSHARED_H
#define TGVSHARED_H

#define TGV_CHUNK_NAME   "tgvideo_frames"
#define TGV_MAGIC        0x54475646    /* 'TGVF' */
#define TGV_VERSION      2

#define TGV_MAX_W        640       /* largest decoded (peer) picture */
#define TGV_MAX_H        960
#define TGV_CAM_W        640       /* largest camera frame (self-view preview) */
#define TGV_CAM_H        480
#define TGV_HDR_SIZE     1024
#define TGV_DEC_CAP      (TGV_MAX_W * TGV_MAX_H * 2)       /* room for 4:2:2 */
#define TGV_PREV_CAP     (TGV_CAM_W * TGV_CAM_H * 3 / 2)
#define TGV_DEC_OFF      TGV_HDR_SIZE
#define TGV_PREV_OFF     (TGV_HDR_SIZE + TGV_DEC_CAP)
#define TGV_CHUNK_SIZE   (TGV_HDR_SIZE + TGV_DEC_CAP + TGV_PREV_CAP)

/* word indexes into the header (TInt32 / unsigned) */
enum {
    TGV_W_MAGIC = 0,
    TGV_W_VERSION,
    TGV_W_STATE,        /* 0 starting, 1 running, 2 idle, 3 error, 4 quitting, 5 order sweep */
    TGV_W_DEC_SEQ,
    TGV_W_DEC_COUNT,    /* decoded pictures put into the chunk */
    TGV_W_DEC_W,
    TGV_W_DEC_H,
    TGV_W_DEC_PATTERN,  /* TYuvSamplingPattern of the decoder output */
    TGV_W_DEC_LAYOUT,   /* TYuvDataLayout of the decoder output */
    TGV_W_DEC_LEN,      /* bytes valid in the decoded buffer */
    TGV_W_PREV_SEQ,
    TGV_W_PREV_COUNT,
    TGV_W_PREV_W,
    TGV_W_PREV_H,
    TGV_W_CAM,          /* camera index in use */
    TGV_W_CAP_FPS,      /* fps x10 */
    TGV_W_ENC_FPS,
    TGV_W_DEC_FPS,
    TGV_W_LAT_MS,       /* last capture -> decoded latency */
    TGV_W_BPS,          /* encoded bytes per second */
    TGV_W_ERRORS,
    TGV_W_DROPS,
    TGV_W_ENC_OK,
    TGV_W_DEC_OK,
    TGV_W_KEYS,         /* IDR units seen */
    TGV_W_ORDER,        /* start order in use + 1 (0 = default, not tested): 1 camera first, 2 decoder first, 3 decoder first + viewfinder */
    TGV_W_BLOCKED,      /* 1 = decoder unavailable while the camera runs */
    TGV_W_SWEEP,        /* 0 never run, 1 order sweep running, 2 done */
    TGV_W_RX_STATE,     /* peer video: 0 none, 1 decoder starting, 2 decoding, 3 decoder setup failed, 4 fed but no pictures, 5 decoder fatal error */
    TGV_W_RX_ERR,       /* last decoder error code (setup / fatal) */
    TGV_W_RX_W,         /* peer SPS size and profile/level */
    TGV_W_RX_H,
    TGV_W_RX_PROFILE,
    TGV_W_RX_LEVEL,
    TGV_W_RX_AU,        /* access units received from the call engine */
    TGV_W_RX_FED,       /* access units given to the decoder */
    TGV_W_RX_DEC,       /* decoder in use: 1 = 0x102073ef, 2 = 0x10204bfc */
    TGV_W_COUNT
};

#endif
