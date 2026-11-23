/*
 * tgvdisp.pyd: draws the frames tgvideo.exe publishes in the global chunk
 * "tgvideo_frames" straight into a PyS60 graphics.Image.
 *
 *   attach() -> 0 or a Symbian error code (negative)
 *   detach()
 *   info() -> tuple of header words (see TGV_W_* in tgvshared.h, in that order) or None
 *   render(image, which, flags=0, last=-1) -> frame counter of the chosen buffer
 *       which: 0 decoded picture, 1 camera preview
 *       flags: 1 swap U and V, 2 swap the byte order of 4:2:2 interleaved data,
 *              4 mirror horizontally (front camera self-view); the picture is also decimated
 *              by 2 (every other pixel and row) when the image is at most half its size
 *       last : if the counter equals it nothing is drawn (frame unchanged)
 *       a negative result is an error: -1 not attached/no header, -2 bad image,
 *       -3 no picture yet or unsupported layout, -4 picture changed while reading
 *
 * YUV -> RGB with integer math and lookup tables built on the heap at attach()
 * time (the only static is the pointer to that block; EPOCALLOWDLLDATA).
 * Output modes: EColor64K (RGB565), EColor16MU / EColor16MA (0xAARRGGBB), EColor16M.
 * A frame is drawn 2x (nearest neighbour) when the image is at least twice its size.
 */
#include <e32std.h>
#include <e32base.h>
#include <fbs.h>
#include <string.h>
#include "Python.h"
#include "tgvshared.h"

/* TYuvSamplingPattern / TYuvDataLayout values of devvideoconstants.h (not included here) */
enum { EYuv420Chroma1 = 1, EYuv420Chroma2 = 2, EYuv420Chroma3 = 4, EYuv422Chroma1 = 8, EYuv422Chroma2 = 0x10 };
enum { EYuvDataPlanar = 1, EYuvDataInterleavedLE = 2, EYuvDataInterleavedBE = 4, EYuvDataSemiPlanar = 8 };

struct TTab
{
    TInt iY[256];       // (y-16)*298 >> 8
    TInt iRV[256];      // 409*(v-128) >> 8
    TInt iGU[256];      // -100*(u-128) >> 8
    TInt iGV[256];      // -208*(v-128) >> 8
    TInt iBU[256];      // 516*(u-128) >> 8
    TUint8 iClip[1024]; // clamp(x - 384) to 0..255
};

static TTab* gTab = NULL;
static RChunk gChunk;
static TBool gAttached = EFalse;
static volatile TInt32* gH = NULL;

/* ------------------------------------------------------------ pixel stores */
static inline TUint16 P565(TInt r, TInt g, TInt b) { return (TUint16)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)); }

struct TSrc
{
    const TUint8* iY; TInt iYStep, iYStride;     // luma row y: iY + y*iYStride, pixel x: + x*iYStep
    const TUint8* iU; const TUint8* iV; TInt iCStep, iCStride, iCShift;   // chroma row: (y >> iCShift)*iCStride; pair p: + p*iCStep
};

// Converts one source row into aDst (mode 0: 565, 1: 32 bit, 2: 24 bit) for aCount pixels, each repeated aRep times.
template <int MODE>
static void Row(const TSrc& s, TInt y, TInt aCount, TInt aRep, TUint8* aDst, TInt aStep, TInt aMirrorW)
{
    const TUint8* yp = s.iY + y * s.iYStride;
    const TUint8* up = s.iU + (y >> s.iCShift) * s.iCStride;
    const TUint8* vp = s.iV + (y >> s.iCShift) * s.iCStride;
    const TTab* t = gTab;
    const TUint8* clip = t->iClip + 384;
    for (TInt xo = 0; xo < aCount; xo++) {
        TInt x = xo * aStep;                       // source column (decimated)
        if (aMirrorW > 0) x = aMirrorW - 1 - x;    // horizontal flip within the drawn width
        TInt yy = t->iY[yp[x * s.iYStep]];
        TInt c = (x >> 1) * s.iCStep;
        TInt u = up[c], v = vp[c];
        TInt r = clip[yy + t->iRV[v]];
        TInt g = clip[yy + t->iGU[u] + t->iGV[v]];
        TInt b = clip[yy + t->iBU[u]];
        for (TInt k = 0; k < aRep; k++) {
            if (MODE == 0) { *(TUint16*)aDst = P565(r, g, b); aDst += 2; }
            else if (MODE == 1) { *(TUint32*)aDst = 0xFF000000u | (TUint32)((r << 16) | (g << 8) | b); aDst += 4; }
            else { aDst[0] = (TUint8)b; aDst[1] = (TUint8)g; aDst[2] = (TUint8)r; aDst += 3; }
        }
    }
}

static TBool BuildSrc(TSrc& s, const TUint8* aBase, TInt aW, TInt aH, TInt aPattern, TInt aLayout, TInt aLen, TInt aFlags)
{
    TInt u = 0, v = 0;
    if (aLayout == EYuvDataPlanar && (aPattern & (EYuv420Chroma1 | EYuv420Chroma2 | EYuv420Chroma3))) {
        if (aLen < aW * aH * 3 / 2) return EFalse;
        s.iY = aBase; s.iYStep = 1; s.iYStride = aW;
        const TUint8* cu = aBase + aW * aH;
        const TUint8* cv = cu + (aW / 2) * (aH / 2);
        if (aFlags & 1) { const TUint8* t = cu; cu = cv; cv = t; }
        s.iU = cu; s.iV = cv; s.iCStep = 1; s.iCStride = aW / 2; s.iCShift = 1;
        return ETrue;
    }
    if (aLayout == EYuvDataSemiPlanar && (aPattern & (EYuv420Chroma1 | EYuv420Chroma2 | EYuv420Chroma3))) {
        if (aLen < aW * aH * 3 / 2) return EFalse;
        s.iY = aBase; s.iYStep = 1; s.iYStride = aW;
        u = 0; v = 1;
        if (aFlags & 1) { u = 1; v = 0; }
        s.iU = aBase + aW * aH + u; s.iV = aBase + aW * aH + v; s.iCStep = 2; s.iCStride = aW; s.iCShift = 1;
        return ETrue;
    }
    if ((aLayout == EYuvDataInterleavedLE || aLayout == EYuvDataInterleavedBE) &&
        (aPattern & (EYuv422Chroma1 | EYuv422Chroma2))) {
        if (aLen < aW * aH * 2) return EFalse;
        // LE: Y0 U Y1 V (YUYV), BE: U Y0 V Y1 (UYVY); flag 2 swaps the two
        TBool uyvy = (aLayout == EYuvDataInterleavedBE);
        if (aFlags & 2) uyvy = !uyvy;
        s.iYStep = 2; s.iYStride = aW * 2; s.iCStep = 4; s.iCStride = aW * 2; s.iCShift = 0;
        if (uyvy) { s.iY = aBase + 1; u = 0; v = 2; } else { s.iY = aBase; u = 1; v = 3; }
        if (aFlags & 1) { TInt t = u; u = v; v = t; }
        s.iU = aBase + u; s.iV = aBase + v;
        return ETrue;
    }
    return EFalse;
}

/* ------------------------------------------------------------ python functions */
static PyObject* tv_attach(PyObject*, PyObject*)
{
    if (gAttached) return Py_BuildValue("i", 0);
    if (!gTab) {
        gTab = (TTab*)User::Alloc(sizeof(TTab));
        if (!gTab) return Py_BuildValue("i", (int)KErrNoMemory);
        for (TInt i = 0; i < 256; i++) {
            TInt y = ((i - 16) * 298) >> 8;
            gTab->iY[i] = y;
            gTab->iRV[i] = (409 * (i - 128)) >> 8;
            gTab->iGU[i] = (-100 * (i - 128)) >> 8;
            gTab->iGV[i] = (-208 * (i - 128)) >> 8;
            gTab->iBU[i] = (516 * (i - 128)) >> 8;
        }
        for (TInt i = 0; i < 1024; i++) {
            TInt v = i - 384;
            gTab->iClip[i] = (TUint8)(v < 0 ? 0 : (v > 255 ? 255 : v));
        }
    }
    _LIT(KName, "tgvideo_frames");
    TInt e = gChunk.OpenGlobal(KName, EFalse);
    if (e != KErrNone) return Py_BuildValue("i", (int)e);
    if (gChunk.Size() < TGV_CHUNK_SIZE) { gChunk.Close(); return Py_BuildValue("i", (int)KErrCorrupt); }
    gH = (volatile TInt32*)gChunk.Base();
    if (gH[TGV_W_MAGIC] != TGV_MAGIC || gH[TGV_W_VERSION] != TGV_VERSION) { gChunk.Close(); gH = NULL; return Py_BuildValue("i", (int)KErrCorrupt); }
    gAttached = ETrue;
    return Py_BuildValue("i", 0);
}

static PyObject* tv_detach(PyObject*, PyObject*)
{
    if (gAttached) { gChunk.Close(); gAttached = EFalse; gH = NULL; }
    if (gTab) { User::Free(gTab); gTab = NULL; }
    Py_INCREF(Py_None);
    return Py_None;
}

static PyObject* tv_info(PyObject*, PyObject*)
{
    if (!gAttached || !gH) { Py_INCREF(Py_None); return Py_None; }
    PyObject* t = PyTuple_New(TGV_W_COUNT);
    if (!t) return NULL;
    for (TInt i = 0; i < TGV_W_COUNT; i++) PyTuple_SetItem(t, i, PyInt_FromLong((long)gH[i]));
    return t;
}

static CFbsBitmap* AsBitmap(PyObject* obj)
{
    PyObject* f = PyObject_GetAttrString(obj, (char*)"_bitmapapi");
    if (!f) { PyErr_Clear(); return NULL; }
    PyObject* c = NULL;
    if (PyCallable_Check(f)) c = PyObject_CallObject(f, NULL);
    Py_DECREF(f);
    if (!c) { PyErr_Clear(); return NULL; }
    CFbsBitmap* bm = NULL;
    if (PyCObject_Check(c)) bm = (CFbsBitmap*)PyCObject_AsVoidPtr(c);
    Py_DECREF(c);
    return bm;
}

static PyObject* tv_render(PyObject*, PyObject* args)
{
    PyObject* img = NULL;
    int which = 0, flags = 0, last = -1;
    if (!PyArg_ParseTuple(args, "Oi|ii", &img, &which, &flags, &last)) return NULL;
    if (!gAttached || !gH || !gTab) return Py_BuildValue("i", -1);
    TInt cnt, seq0, w, h, pat = EYuv420Chroma1, lay = EYuvDataPlanar, len;
    const TUint8* base;
    if (which == 0) {
        seq0 = gH[TGV_W_DEC_SEQ]; cnt = gH[TGV_W_DEC_COUNT];
        w = gH[TGV_W_DEC_W]; h = gH[TGV_W_DEC_H]; pat = gH[TGV_W_DEC_PATTERN]; lay = gH[TGV_W_DEC_LAYOUT];
        len = gH[TGV_W_DEC_LEN];
        base = (const TUint8*)gChunk.Base() + TGV_DEC_OFF;
    } else {
        seq0 = gH[TGV_W_PREV_SEQ]; cnt = gH[TGV_W_PREV_COUNT];
        w = gH[TGV_W_PREV_W]; h = gH[TGV_W_PREV_H];
        len = w * h * 3 / 2;
        if (w > TGV_CAM_W || h > TGV_CAM_H) return Py_BuildValue("i", -3);
        base = (const TUint8*)gChunk.Base() + TGV_PREV_OFF;
    }
    if (cnt == 0 || w < 2 || h < 2 || w > TGV_MAX_W || h > TGV_MAX_H) return Py_BuildValue("i", -3);
    if (cnt == last) return Py_BuildValue("i", (int)cnt);
    if (seq0 & 1) return Py_BuildValue("i", -4);
    w &= ~1;
    TSrc s;
    if (!BuildSrc(s, base, w, h, pat, lay, len, flags)) return Py_BuildValue("i", -3);

    CFbsBitmap* bm = AsBitmap(img);
    if (!bm) return Py_BuildValue("i", -2);
    TSize sz = bm->SizeInPixels();
    TDisplayMode dm = bm->DisplayMode();
    TInt mode, bpp;
    if (dm == EColor64K) { mode = 0; bpp = 2; }
    else if (dm == EColor16MU || dm == EColor16MA) { mode = 1; bpp = 4; }
    else if (dm == EColor16M) { mode = 2; bpp = 3; }
    else return Py_BuildValue("i", -2);
    TInt scale = (sz.iWidth >= 2 * w && sz.iHeight >= 2 * h) ? 2 : 1;
    TInt step = 1;
    if (scale == 1 && sz.iWidth * 2 <= w && sz.iHeight * 2 <= h) step = 2;
    TInt cw = w, ch = h;
    if (step == 2) { cw = w / 2; ch = h / 2; }
    if (cw * scale > sz.iWidth) cw = sz.iWidth / scale;
    if (ch * scale > sz.iHeight) ch = sz.iHeight / scale;
    TInt mirrorW = (flags & 4) ? cw * step : 0;
    TInt stride = CFbsBitmap::ScanLineLength(sz.iWidth, dm);
    bm->LockHeap();
    TUint8* dst = (TUint8*)bm->DataAddress();
    if (dst) {
        for (TInt y = 0; y < ch; y++) {
            TUint8* row = dst + y * scale * stride;
            if (mode == 0) Row<0>(s, y * step, cw, scale, row, step, mirrorW);
            else if (mode == 1) Row<1>(s, y * step, cw, scale, row, step, mirrorW);
            else Row<2>(s, y * step, cw, scale, row, step, mirrorW);
            if (scale == 2) Mem::Copy(row + stride, row, cw * 2 * bpp);
        }
    }
    bm->UnlockHeap();
    if (gH[(which == 0) ? TGV_W_DEC_SEQ : TGV_W_PREV_SEQ] != seq0) return Py_BuildValue("i", -4);
    return Py_BuildValue("i", (int)cnt);
}

static const PyMethodDef tv_methods[] = {
    {"attach", (PyCFunction)tv_attach, METH_VARARGS, NULL},
    {"detach", (PyCFunction)tv_detach, METH_VARARGS, NULL},
    {"info", (PyCFunction)tv_info, METH_VARARGS, NULL},
    {"render", (PyCFunction)tv_render, METH_VARARGS, NULL},
    {NULL, NULL, 0, NULL}
};

extern "C" {
DL_EXPORT(void) inittgvdisp(void)
{
    Py_InitModule("tgvdisp", (PyMethodDef*)tv_methods);
}
}
