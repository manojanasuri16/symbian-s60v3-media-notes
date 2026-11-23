/*
 * tgvidprobe.exe: reports what video hardware the phone offers (DevVideo
 * decoders/encoders/pre/post processors, MMF video controllers, cameras,
 * display). Every step is logged before it runs and wrapped on its own, so a
 * failure only costs that step. Output: E:\retrotele_videoprobe.txt, falling
 * back to C:\Data\. The last line is always "probe end".
 */
#include <e32base.h>
#include <e32std.h>
#include <f32file.h>
#include <hal.h>
#include <badesca.h>
#include <w32std.h>
#include <fbs.h>
#include <ecam.h>
#include <mmf/devvideo/devvideoplay.h>
#include <mmf/devvideo/devvideorecord.h>
#include <mmf/common/mmfcontrollerpluginresolver.h>

static RFs gFs;
static RFile gFile;
static TBool gOpen = EFalse;

static void Log(TRefByValue<const TDesC8> aFmt, ...)
{
    VA_LIST l;
    VA_START(l, aFmt);
    TBuf8<400> b;
    b.FormatList(aFmt, l);
    VA_END(l);
    b.Append('\n');
    if (gOpen) { gFile.Write(b); gFile.Flush(); }
}

static void N8(const TDesC16& aIn, TBuf8<80>& aOut)
{
    aOut.Zero();
    for (TInt i = 0; i < aIn.Length() && i < 78; i++) {
        TUint c = aIn[i];
        aOut.Append((c >= 32 && c < 127) ? c : '?');
    }
}
static void M8(const TDesC8& aIn, TBuf8<80>& aOut)
{
    aOut.Zero();
    for (TInt i = 0; i < aIn.Length() && i < 78; i++) {
        TUint c = aIn[i];
        aOut.Append((c >= 32 && c < 127) ? c : '?');
    }
}
#define X100(r) ((TInt)((r) * 100))

// ---- waiting helper (active scheduler loop with a timer) ----
class CPump : public CBase
{
public:
    static CPump* NewL()
    {
        CPump* p = new (ELeave) CPump;
        CleanupStack::PushL(p);
        p->iTimer = CPeriodic::NewL(CActive::EPriorityLow);
        CleanupStack::Pop();
        return p;
    }
    ~CPump() { delete iTimer; }
    // Runs the scheduler until aMs elapsed or *aFlag != 0 (aFlag may be NULL).
    void Run(TInt aMs, volatile TInt* aFlag)
    {
        iDeadline = aMs / 20;
        iTicks = 0;
        iFlag = aFlag;
        iTimer->Start(20000, 20000, TCallBack(Tick, this));
        iWait.Start();
        iTimer->Cancel();
    }
private:
    static TInt Tick(TAny* a)
    {
        CPump* s = (CPump*)a;
        if (++s->iTicks >= s->iDeadline || (s->iFlag && *s->iFlag))
            s->iWait.AsyncStop();
        return 0;
    }
    CPeriodic* iTimer;
    CActiveSchedulerWait iWait;
    TInt iDeadline, iTicks;
    volatile TInt* iFlag;
};
static CPump* gPump;

// ---- 1. DevVideo ----
class CPlayObs : public CBase, public MMMFDevVideoPlayObserver
{
public:
    void MdvpoNewBuffers() {}
    void MdvpoReturnPicture(TVideoPicture*) {}
    void MdvpoSupplementalInformation(const TDesC8&, const TTimeIntervalMicroSeconds&, const TPictureId&) {}
    void MdvpoPictureLoss() {}
    void MdvpoPictureLoss(const TArray<TPictureId>&) {}
    void MdvpoSliceLoss(TUint, TUint, const TPictureId&) {}
    void MdvpoReferencePictureSelection(const TDesC8&) {}
    void MdvpoTimedSnapshotComplete(TInt, TPictureData*, const TTimeIntervalMicroSeconds&, const TPictureId&) {}
    void MdvpoNewPictures() {}
    void MdvpoFatalError(TInt aErr) { Log(_L8("play observer fatal %d"), aErr); }
    void MdvpoInitComplete(TInt aErr) { Log(_L8("play observer init %d"), aErr); }
    void MdvpoStreamEnd() {}
};
class CRecObs : public CBase, public MMMFDevVideoRecordObserver
{
public:
    void MdvroReturnPicture(TVideoPicture*) {}
    void MdvroSupplementalInfoSent() {}
    void MdvroNewBuffers() {}
    void MdvroFatalError(TInt aErr) { Log(_L8("rec observer fatal %d"), aErr); }
    void MdvroInitializeComplete(TInt aErr) { Log(_L8("rec observer init %d"), aErr); }
    void MdvroStreamEnd() {}
};

static const TText8* const KMimes[] = {
    (const TText8*)"video/H264",
    (const TText8*)"video/H264; profile-level-id=42800A",
    (const TText8*)"video/H264; profile-level-id=42E00A",
    (const TText8*)"video/H264; profile-level-id=64001F",
    (const TText8*)"video/mp4v-es",
    (const TText8*)"video/mp4v-es; profile-level-id=3",
    (const TText8*)"video/H263-2000",
    (const TText8*)"video/H263-2000; profile=0; level=10",
    (const TText8*)"video/H263",
    (const TText8*)"video/x-vp8",
    (const TText8*)"video/vp8",
    (const TText8*)"video/h265",
    0 };

static void LogFormats(const TDesC8& aTag, const RPointerArray<CCompressedVideoFormat>& a)
{
    TBuf8<80> m;
    for (TInt i = 0; i < a.Count(); i++) {
        M8(a[i]->MimeType(), m);
        Log(_L8("  %S fmt %S"), &aTag, &m);
    }
}
static void LogPictureRates(const RArray<TPictureRateAndSize>& a)
{
    for (TInt i = 0; i < a.Count(); i++)
        Log(_L8("  maxrate %dx%d @ %d/100 fps"), a[i].iPictureSize.iWidth,
            a[i].iPictureSize.iHeight, X100(a[i].iPictureRate));
}
static void LogUncompressed(const TDesC8& aTag, const RArray<TUncompressedVideoFormat>& a)
{
    for (TInt i = 0; i < a.Count(); i++) {
        const TUncompressedVideoFormat& f = a[i];
        if (f.iDataFormat == EYuvRawData)
            Log(_L8("  %S yuv coeff=%d pattern=%d layout=%d"), &aTag, (TInt)f.iYuvFormat.iCoefficients,
                (TInt)f.iYuvFormat.iPattern, (TInt)f.iYuvFormat.iDataLayout);
        else
            Log(_L8("  %S datafmt=%d rgb=0x%x"), &aTag, (TInt)f.iDataFormat, (TInt)f.iRgbFormat);
    }
}

static void DecoderInfoL(CMMFDevVideoPlay* aPlay, TUid aUid)
{
    CVideoDecoderInfo* d = aPlay->VideoDecoderInfoLC(aUid);
    TBuf8<80> s;
    N8(d->Manufacturer(), s);
    Log(_L8("  decoder 0x%08x manufacturer=%S"), aUid.iUid, &s);
    N8(d->Identifier(), s);
    TVersion v = d->Version();
    Log(_L8("  id=%S version=%d.%d.%d accelerated=%d directdisplay=%d"), &s, (TInt)v.iMajor,
        (TInt)v.iMinor, (TInt)v.iBuild, (TInt)d->Accelerated(), (TInt)d->SupportsDirectDisplay());
    Log(_L8("  maxpic=%dx%d maxbitrate=%u pictureloss=%d sliceloss=%d"), d->MaxPictureSize().iWidth,
        d->MaxPictureSize().iHeight, d->MaxBitrate(), (TInt)d->SupportsPictureLoss(),
        (TInt)d->SupportsSliceLoss());
    LogPictureRates(d->MaxPictureRates());
    LogFormats(_L8("decoder"), d->SupportedFormats());
    CleanupStack::PopAndDestroy(d);
}
static void PostInfoL(CMMFDevVideoPlay* aPlay, TUid aUid)
{
    CPostProcessorInfo* d = aPlay->PostProcessorInfoLC(aUid);
    TBuf8<80> s;
    N8(d->Manufacturer(), s);
    Log(_L8("  postproc 0x%08x manufacturer=%S accelerated=%d directdisplay=%d"), aUid.iUid, &s,
        (TInt)d->Accelerated(), (TInt)d->SupportsDirectDisplay());
    Log(_L8("  combos=%d rotations=0x%x arbscale=%d scalefactors=%d"), d->SupportedCombinations().Count(),
        (TInt)d->SupportedRotations(), (TInt)d->SupportsArbitraryScaling(),
        d->SupportedScaleFactors().Count());
    for (TInt i = 0; i < d->SupportedCombinations().Count(); i++)
        Log(_L8("  combo 0x%x"), (TInt)d->SupportedCombinations()[i]);
    LogUncompressed(_L8("postproc in"), d->SupportedFormats());
    CleanupStack::PopAndDestroy(d);
}
static void EncoderInfoL(CMMFDevVideoRecord* aRec, TUid aUid)
{
    CVideoEncoderInfo* d = aRec->VideoEncoderInfoLC(aUid);
    TBuf8<80> s;
    N8(d->Manufacturer(), s);
    Log(_L8("  encoder 0x%08x manufacturer=%S"), aUid.iUid, &s);
    N8(d->Identifier(), s);
    TVersion v = d->Version();
    Log(_L8("  id=%S version=%d.%d.%d accelerated=%d directcapture=%d"), &s, (TInt)v.iMajor,
        (TInt)v.iMinor, (TInt)v.iBuild, (TInt)d->Accelerated(), (TInt)d->SupportsDirectCapture());
    Log(_L8("  maxpic=%dx%d maxbitrate=%u bitratelayers=%u dataunittypes=0x%x encaps=0x%x"),
        d->MaxPictureSize().iWidth, d->MaxPictureSize().iHeight, d->MaxBitrate(), d->MaxBitrateLayers(),
        (TInt)d->SupportedDataUnitTypes(), (TInt)d->SupportedDataUnitEncapsulations());
    Log(_L8("  pictureoptions=0x%x uep=%u scalability=%u pictureloss=%d sliceloss=%d"),
        (TInt)d->SupportedPictureOptions(), d->MaxUnequalErrorProtectionLevels(),
        d->MaxInLayerScalabilitySteps(), (TInt)d->SupportsPictureLoss(), (TInt)d->SupportsSliceLoss());
    LogPictureRates(d->MaxPictureRates());
    LogUncompressed(_L8("encoder in"), d->SupportedInputFormats());
    LogFormats(_L8("encoder out"), d->SupportedOutputFormats());
    CleanupStack::PopAndDestroy(d);
}
static void PreInfoL(CMMFDevVideoRecord* aRec, TUid aUid)
{
    CPreProcessorInfo* d = aRec->PreProcessorInfoLC(aUid);
    TBuf8<80> s;
    N8(d->Manufacturer(), s);
    Log(_L8("  preproc 0x%08x manufacturer=%S accelerated=%d directcapture=%d"), aUid.iUid, &s,
        (TInt)d->Accelerated(), (TInt)d->SupportsDirectCapture());
    Log(_L8("  combos=%d rotations=0x%x"), d->SupportedCombinations().Count(), (TInt)d->SupportedRotations());
    LogUncompressed(_L8("preproc in"), d->SupportedInputFormats());
    LogUncompressed(_L8("preproc out"), d->SupportedOutputFormats());
    CleanupStack::PopAndDestroy(d);
}

static void ProbePlayL()
{
    CPlayObs* obs = new (ELeave) CPlayObs;
    CleanupStack::PushL(obs);
    Log(_L8("step devvideoplay NewL"));
    CMMFDevVideoPlay* play = CMMFDevVideoPlay::NewL(*obs);
    CleanupStack::PushL(play);
    for (TInt exact = 1; exact >= 0; exact--) {
        for (TInt i = 0; KMimes[i]; i++) {
            TPtrC8 mime(KMimes[i]);
            TBuf8<80> m; M8(mime, m);
            Log(_L8("step FindDecodersL mime=%S exact=%d"), &m, exact);
            RArray<TUid> uids;
            TRAPD(err, play->FindDecodersL(mime, 0, uids, exact));
            Log(_L8("decoders mime=%S exact=%d err=%d count=%d"), &m, exact, err, uids.Count());
            if (exact) {
                for (TInt j = 0; j < uids.Count(); j++) {
                    Log(_L8("step DecoderInfo 0x%08x"), uids[j].iUid);
                    TRAPD(e2, DecoderInfoL(play, uids[j]));
                    if (e2) Log(_L8("  decoderinfo err=%d"), e2);
                }
            }
            uids.Close();
            if (exact == 0 && i >= 3) break;   // loose matching: only H264 variants
        }
    }
    {
        RArray<TUid> pp;
        Log(_L8("step GetPostProcessorListL"));
        TRAPD(err, play->GetPostProcessorListL(pp));
        Log(_L8("postprocessors err=%d count=%d"), err, pp.Count());
        for (TInt j = 0; j < pp.Count(); j++) {
            TRAPD(e2, PostInfoL(play, pp[j]));
            if (e2) Log(_L8("  postinfo err=%d"), e2);
        }
        pp.Close();
        Log(_L8("step FindPostProcessorsL any"));
        RArray<TUid> pp2;
        TRAPD(e3, play->FindPostProcessorsL(0, pp2));
        Log(_L8("FindPostProcessorsL(0) err=%d count=%d"), e3, pp2.Count());
        pp2.Close();
    }
    CleanupStack::PopAndDestroy(2);
}

static void ProbeRecordL()
{
    CRecObs* obs = new (ELeave) CRecObs;
    CleanupStack::PushL(obs);
    Log(_L8("step devvideorecord NewL"));
    CMMFDevVideoRecord* rec = CMMFDevVideoRecord::NewL(*obs);
    CleanupStack::PushL(rec);
    for (TInt exact = 1; exact >= 0; exact--) {
        for (TInt i = 0; KMimes[i]; i++) {
            TPtrC8 mime(KMimes[i]);
            TBuf8<80> m; M8(mime, m);
            Log(_L8("step FindEncodersL mime=%S exact=%d"), &m, exact);
            RArray<TUid> uids;
            TRAPD(err, rec->FindEncodersL(mime, 0, uids, exact));
            Log(_L8("encoders mime=%S exact=%d err=%d count=%d"), &m, exact, err, uids.Count());
            if (exact) {
                for (TInt j = 0; j < uids.Count(); j++) {
                    Log(_L8("step EncoderInfo 0x%08x"), uids[j].iUid);
                    TRAPD(e2, EncoderInfoL(rec, uids[j]));
                    if (e2) Log(_L8("  encoderinfo err=%d"), e2);
                }
            }
            uids.Close();
            if (exact == 0 && i >= 3) break;
        }
    }
    {
        RArray<TUid> pp;
        Log(_L8("step GetPreProcessorListL"));
        TRAPD(err, rec->GetPreProcessorListL(pp));
        Log(_L8("preprocessors err=%d count=%d"), err, pp.Count());
        for (TInt j = 0; j < pp.Count(); j++) {
            TRAPD(e2, PreInfoL(rec, pp[j]));
            if (e2) Log(_L8("  preinfo err=%d"), e2);
        }
        pp.Close();
        RArray<TUid> pp2;
        Log(_L8("step FindPreProcessorsL any"));
        TRAPD(e3, rec->FindPreProcessorsL(0, pp2));
        Log(_L8("FindPreProcessorsL(0) err=%d count=%d"), e3, pp2.Count());
        pp2.Close();
    }
    CleanupStack::PopAndDestroy(2);
}

// ---- 2. MMF controllers ----
static void LogFmtInfo(const TDesC8& aTag, const RMMFFormatImplInfoArray& a)
{
    for (TInt i = 0; i < a.Count(); i++) {
        CMMFFormatImplementationInformation* f = a[i];
        TBuf8<80> n;
        N8(f->DisplayName(), n);
        Log(_L8("  %S format 0x%08x %S"), &aTag, f->Uid().iUid, &n);
        const CDesC8Array& mt = f->SupportedMimeTypes();
        for (TInt k = 0; k < mt.MdcaCount(); k++) {
            M8(mt.MdcaPoint(k), n);
            Log(_L8("    mime %S"), &n);
        }
        const CDesC8Array& ex = f->SupportedFileExtensions();
        for (TInt k = 0; k < ex.MdcaCount(); k++) {
            M8(ex.MdcaPoint(k), n);
            Log(_L8("    ext %S"), &n);
        }
    }
}
static void ProbeControllersL()
{
    Log(_L8("step list all controllers"));
    CMMFControllerPluginSelectionParameters* p = CMMFControllerPluginSelectionParameters::NewLC();
    RMMFControllerImplInfoArray all;
    CleanupResetAndDestroyPushL(all);
    p->ListImplementationsL(all);
    Log(_L8("controllers total=%d"), all.Count());
    for (TInt i = 0; i < all.Count(); i++) {
        CMMFControllerImplementationInformation* c = all[i];
        TBuf8<80> n, s;
        N8(c->DisplayName(), n);
        N8(c->Supplier(), s);
        Log(_L8("controller 0x%08x name=%S supplier=%S play=%d record=%d"), c->Uid().iUid, &n, &s,
            c->PlayFormats().Count(), c->RecordFormats().Count());
        LogFmtInfo(_L8("play"), c->PlayFormats());
        LogFmtInfo(_L8("record"), c->RecordFormats());
    }
    CleanupStack::PopAndDestroy(&all);
    CleanupStack::PopAndDestroy(p);

    static const char* const KChk[] = { "video/H264", "video/mp4v-es", "video/H263-2000", 0 };
    for (TInt m = 0; KChk[m]; m++) {
        TPtrC8 mime((const TUint8*)KChk[m]);
        TBuf8<80> mm; M8(mime, mm);
        for (TInt rec = 0; rec < 2; rec++) {
            Log(_L8("step controller select mime=%S %s"), &mm, rec ? "record" : "play");
            CMMFControllerPluginSelectionParameters* q = CMMFControllerPluginSelectionParameters::NewLC();
            CMMFFormatSelectionParameters* f = CMMFFormatSelectionParameters::NewLC();
            f->SetMatchToMimeTypeL(mime);
            if (rec) q->SetRequiredRecordFormatSupportL(*f);
            else q->SetRequiredPlayFormatSupportL(*f);
            RMMFControllerImplInfoArray r;
            CleanupResetAndDestroyPushL(r);
            q->ListImplementationsL(r);
            Log(_L8("mime %S %s controllers=%d"), &mm, rec ? "record" : "play", r.Count());
            for (TInt i = 0; i < r.Count(); i++) {
                TBuf8<80> n;
                N8(r[i]->DisplayName(), n);
                Log(_L8("  match controller 0x%08x %S"), r[i]->Uid().iUid, &n);
            }
            CleanupStack::PopAndDestroy(3, q);
        }
    }
}

// ---- 3. Camera ----
class CCamProbe : public CBase, public MCameraObserver
{
public:
    volatile TInt iReserved, iPowered, iReserveErr, iPowerErr;
    TInt iVf, iVid;
    void ReserveComplete(TInt aErr) { iReserveErr = aErr; iReserved = 1; }
    void PowerOnComplete(TInt aErr) { iPowerErr = aErr; iPowered = 1; }
    void ViewFinderFrameReady(CFbsBitmap&) { iVf++; }
    void ImageReady(CFbsBitmap* aB, HBufC8* aD, TInt) { delete aB; delete aD; }
    void FrameBufferReady(MFrameBuffer* aB, TInt aErr)
    {
        if (aErr == KErrNone) iVid++;
        else iLastErr = aErr;
        if (aB) aB->Release();
    }
    TInt iLastErr;
};

struct TFmtName { CCamera::TFormat f; const char* n; };
static const TFmtName KFmts[] = {
    { CCamera::EFormatYUV420Planar, "YUV420Planar" },
    { CCamera::EFormatYUV420Interleaved, "YUV420Interleaved" },
    { CCamera::EFormatYUV420SemiPlanar, "YUV420SemiPlanar" },
    { CCamera::EFormatYUV422, "YUV422" },
    { CCamera::EFormatYUV422Reversed, "YUV422Reversed" },
    { CCamera::EFormatJpeg, "Jpeg" },
    { CCamera::EFormatExif, "Exif" },
    { CCamera::EFormatFbsBitmapColor16M, "FbsColor16M" },
    { CCamera::EFormatFbsBitmapColor64K, "FbsColor64K" },
    { CCamera::EFormatFbsBitmapColor4K, "FbsColor4K" },
    { CCamera::EFormat16BitRGB565, "RGB565" },
    { CCamera::EFormat32BitRGB888, "RGB888" },
    { (CCamera::TFormat)0, 0 } };

static void ProbeOneCameraL(TInt aIdx)
{
    CCamProbe* obs = new (ELeave) CCamProbe;
    CleanupStack::PushL(obs);
    Log(_L8("step camera %d NewL"), aIdx);
    CCamera* cam = CCamera::NewL(*obs, aIdx);
    CleanupStack::PushL(cam);
    Log(_L8("step camera %d Reserve"), aIdx);
    cam->Reserve();
    gPump->Run(5000, &obs->iReserved);
    Log(_L8("camera %d reserve done=%d err=%d"), aIdx, (TInt)obs->iReserved, obs->iReserveErr);
    if (!obs->iReserved || obs->iReserveErr) { CleanupStack::PopAndDestroy(2); return; }
    Log(_L8("step camera %d PowerOn"), aIdx);
    cam->PowerOn();
    gPump->Run(5000, &obs->iPowered);
    Log(_L8("camera %d poweron done=%d err=%d"), aIdx, (TInt)obs->iPowered, obs->iPowerErr);
    if (!obs->iPowered || obs->iPowerErr) { cam->Release(); CleanupStack::PopAndDestroy(2); return; }

    TCameraInfo ci;
    cam->CameraInfo(ci);
    Log(_L8("camera %d orientation=%d options=0x%x flash=0x%x exposure=0x%x wb=0x%x"), aIdx,
        (TInt)ci.iOrientation, (TInt)ci.iOptionsSupported, (TInt)ci.iFlashModesSupported,
        (TInt)ci.iExposureModesSupported, (TInt)ci.iWhiteBalanceModesSupported);
    Log(_L8("  zoom=%d..%d digzoom=%d imagesizes=%d imageformats=0x%x"), ci.iMinZoom, ci.iMaxZoom,
        ci.iMaxDigitalZoom, ci.iNumImageSizesSupported, (TInt)ci.iImageFormatsSupported);
    Log(_L8("  videosizes=%d videorates=%d videoformats=0x%x maxframesperbuf=%d maxbuffers=%d"),
        ci.iNumVideoFrameSizesSupported, ci.iNumVideoFrameRatesSupported,
        (TInt)ci.iVideoFrameFormatsSupported, ci.iMaxFramesPerBufferSupported, ci.iMaxBuffersSupported);

    // enumerate per format; remember best candidate for capture tests
    TInt candFmt = -1;
    for (TInt fi = 0; KFmts[fi].n; fi++) {
        TBool vid = ci.iVideoFrameFormatsSupported & KFmts[fi].f;
        TBool img = ci.iImageFormatsSupported & KFmts[fi].f;
        Log(_L8("camera %d format %s video=%d image=%d"), aIdx, KFmts[fi].n, (TInt)vid, (TInt)img);
        if (img) {
            for (TInt s = 0; s < ci.iNumImageSizesSupported; s++) {
                TSize sz;
                cam->EnumerateCaptureSizes(sz, s, KFmts[fi].f);
                Log(_L8("  image size[%d]=%dx%d"), s, sz.iWidth, sz.iHeight);
            }
        }
        if (!vid) continue;
        if (candFmt < 0) candFmt = fi;
        for (TInt s = 0; s < ci.iNumVideoFrameSizesSupported; s++) {
            TSize sz;
            cam->EnumerateVideoFrameSizes(sz, s, KFmts[fi].f);
            Log(_L8("  video size[%d]=%dx%d"), s, sz.iWidth, sz.iHeight);
            for (TInt r = 0; r < ci.iNumVideoFrameRatesSupported; r++) {
                TReal32 rate = 0;
                cam->EnumerateVideoFrameRates(rate, r, KFmts[fi].f, s);
                Log(_L8("    rate[%d]=%d/100 fps"), r, X100(rate));
            }
        }
    }

    // viewfinder bitmaps at two sizes
    static const TInt KVf[2][2] = { { 160, 120 }, { 320, 240 } };
    for (TInt k = 0; k < 2; k++) {
        TSize sz(KVf[k][0], KVf[k][1]);
        obs->iVf = 0;
        Log(_L8("step camera %d viewfinderbitmaps %dx%d"), aIdx, sz.iWidth, sz.iHeight);
        TRAPD(err, cam->StartViewFinderBitmapsL(sz));
        if (err) { Log(_L8("viewfinderbitmaps %dx%d err=%d"), KVf[k][0], KVf[k][1], err); continue; }
        gPump->Run(3000, NULL);
        cam->StopViewFinder();
        Log(_L8("viewfinderbitmaps req %dx%d got %dx%d frames=%d in 3s (%d/10 fps)"), KVf[k][0], KVf[k][1],
            sz.iWidth, sz.iHeight, obs->iVf, obs->iVf * 10 / 3);
    }
    Log(_L8("viewfinder direct: skipped (needs a window; probe is a non-UI exe)"));

    // video capture on the first video format, smallest and largest size, highest rate
    for (TInt fi = 0; KFmts[fi].n; fi++) {
        if (!(ci.iVideoFrameFormatsSupported & KFmts[fi].f)) continue;
        if (ci.iNumVideoFrameSizesSupported < 1) break;
        TInt small = 0, large = 0, smallA = 0x7fffffff, largeA = -1;
        for (TInt s = 0; s < ci.iNumVideoFrameSizesSupported; s++) {
            TSize sz; cam->EnumerateVideoFrameSizes(sz, s, KFmts[fi].f);
            TInt a = sz.iWidth * sz.iHeight;
            if (a < smallA) { smallA = a; small = s; }
            if (a > largeA) { largeA = a; large = s; }
        }
        TInt picks[2] = { small, large };
        for (TInt k = 0; k < 2; k++) {
            if (k == 1 && large == small) break;
            TSize sz; cam->EnumerateVideoFrameSizes(sz, picks[k], KFmts[fi].f);
            TInt bestR = 0; TReal32 bestV = 0;
            for (TInt r = 0; r < ci.iNumVideoFrameRatesSupported; r++) {
                TReal32 rate = 0;
                cam->EnumerateVideoFrameRates(rate, r, KFmts[fi].f, picks[k]);
                if (rate > bestV) { bestV = rate; bestR = r; }
            }
            obs->iVid = 0; obs->iLastErr = 0;
            Log(_L8("step camera %d videocapture %s %dx%d rateidx=%d (%d/100 fps)"), aIdx, KFmts[fi].n,
                sz.iWidth, sz.iHeight, bestR, X100(bestV));
            TRAPD(err, cam->PrepareVideoCaptureL(KFmts[fi].f, picks[k], bestR, 2, 1));
            if (err) { Log(_L8("videocapture prepare err=%d"), err); continue; }
            cam->StartVideoCapture();
            gPump->Run(3000, NULL);
            cam->StopVideoCapture();
            Log(_L8("videocapture %s %dx%d frames=%d in 3s (%d/10 fps) lasterr=%d"), KFmts[fi].n, sz.iWidth,
                sz.iHeight, obs->iVid, obs->iVid * 10 / 3, obs->iLastErr);
        }
        break;   // first video format only
    }
    cam->PowerOff();
    cam->Release();
    CleanupStack::PopAndDestroy(2);
}

static void ProbeCamerasL()
{
    Log(_L8("step CamerasAvailable"));
    TInt n = CCamera::CamerasAvailable();
    Log(_L8("cameras available=%d"), n);
    for (TInt i = 0; i < n; i++) {
        TRAPD(err, ProbeOneCameraL(i));
        Log(_L8("camera %d probe end err=%d"), i, err);
    }
}

// ---- 4. display / system ----
static void ProbeDisplayL()
{
    TInt v;
    TInt e;
    e = HAL::Get(HAL::EMemoryRAMFree, v);
    Log(_L8("ram free=%d err=%d"), v, e);
    e = HAL::Get(HAL::EMemoryRAM, v);
    Log(_L8("ram total=%d err=%d"), v, e);
    e = HAL::Get(HAL::ECPUSpeed, v);
    Log(_L8("cpu speed=%d kHz err=%d"), v, e);
    e = HAL::Get(HAL::EDisplayXPixels, v);
    TInt w = v;
    HAL::Get(HAL::EDisplayYPixels, v);
    Log(_L8("hal display=%dx%d"), w, v);
    TVersion ver = User::Version();
    Log(_L8("os version=%d.%d build %d"), (TInt)ver.iMajor, (TInt)ver.iMinor, (TInt)ver.iBuild);
    TInt mach;
    HAL::Get(HAL::EMachineUid, mach);
    Log(_L8("machine uid=0x%08x"), mach);
    Log(_L8("step window server"));
    RWsSession ws;
    User::LeaveIfError(ws.Connect());
    CleanupClosePushL(ws);
    CWsScreenDevice* sd = new (ELeave) CWsScreenDevice(ws);
    CleanupStack::PushL(sd);
    User::LeaveIfError(sd->Construct());
    TSize sz = sd->SizeInPixels();
    Log(_L8("screen size=%dx%d mode=%d"), sz.iWidth, sz.iHeight, (TInt)sd->DisplayMode());
    TDisplayMode modes[] = { EGray2, EGray4, EGray16, EGray256, EColor16, EColor256, EColor64K,
                             EColor16M, EColor4K, EColor16MU, EColor16MA };
    for (TUint i = 0; i < sizeof(modes) / sizeof(modes[0]); i++) {
        CFbsBitmap* bm = new CFbsBitmap;
        TInt ce = bm ? bm->Create(TSize(2, 2), modes[i]) : KErrNoMemory;
        delete bm;
        Log(_L8("  bitmap displaymode %d create err=%d"), (TInt)modes[i], ce);
    }
    Log(_L8("  (EColor16MA=%d EColor16MU=%d EColor64K=%d); current mode listed above"), (TInt)EColor16MA,
        (TInt)EColor16MU, (TInt)EColor64K);
    CleanupStack::PopAndDestroy(2);
}

static void Step(const char* aName, void (*aFn)())
{
    Log(_L8("=== step %s begin"), aName);
    TRAPD(err, aFn());
    Log(_L8("=== step %s end err=%d"), aName, err);
}

static void RunL()
{
    User::LeaveIfError(gFs.Connect());
    _LIT(KCard, "E:\\retrotele_videoprobe.txt");
    _LIT(KPhone, "C:\\Data\\retrotele_videoprobe.txt");
    gFs.MkDirAll(KPhone);
    if (gFile.Replace(gFs, KCard, EFileWrite | EFileShareAny) != KErrNone)
        User::LeaveIfError(gFile.Replace(gFs, KPhone, EFileWrite | EFileShareAny));
    gOpen = ETrue;
    Log(_L8("probe start"));
    CActiveScheduler* sch = new (ELeave) CActiveScheduler;
    CleanupStack::PushL(sch);
    CActiveScheduler::Install(sch);
    gPump = CPump::NewL();
    CleanupStack::PushL(gPump);
    TInt e = RFbsSession::Connect();
    Log(_L8("fbs connect err=%d"), e);
    Step("display", ProbeDisplayL);
    Step("devvideo_play", ProbePlayL);
    Step("devvideo_record", ProbeRecordL);
    Step("controllers", ProbeControllersL);
    Step("cameras", ProbeCamerasL);
    Log(_L8("probe end"));
    CleanupStack::PopAndDestroy(2);
}

GLDEF_C TInt E32Main()
{
    CTrapCleanup* cleanup = CTrapCleanup::New();
    TRAPD(err, RunL());
    if (err && gOpen) Log(_L8("fatal err=%d"), err);
    if (gOpen) { Log(_L8("probe end")); gFile.Close(); }
    gFs.Close();
    delete cleanup;
    return err;
}
