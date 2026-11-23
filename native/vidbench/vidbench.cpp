/*
 * tgvbench.exe: live H.264 benchmark for video calls on the Nokia N79
 * (DevVideo encode/decode + ECam capture + idle-thread CPU headroom).
 *
 *   TEST 1  encoder only, synthetic YUV420 frames (several sizes/bitrates)
 *   TEST 2  decoder fed with the stream TEST 1 produced (flat out, per config)
 *   TEST 3  camera 0/1 -> encoder -> decoder (decode paced at capture timing)
 *
 * Every step is logged BEFORE it runs and flushed. Output: E:\retrotele_videobench.txt,
 * falling back to C:\Data\. The last line is always "bench end".
 * Times are milliseconds; "x100" values are fixed point (1234 = 12.34).
 */
#include <e32base.h>
#include <e32std.h>
#include <e32math.h>
#include <f32file.h>
#include <hal.h>
#include <badesca.h>
#include <ecam.h>
#include <mmf/devvideo/devvideoplay.h>
#include <mmf/devvideo/devvideorecord.h>
#include "clip_data.h"

static RFs gFs;
static RFile gFile;
static TBool gOpen = EFalse;
static volatile TInt gAbort = 0;

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

// ---- time ----
static TInt gFastDown = 0;
static TInt gFastFreq = 0;      // FastCounter frequency, 0 = use NTickCount
static TInt gTickUs = 1000;
static void InitClock()
{
    TInt f = 0;
    if (HAL::Get(HAL::EFastCounterFrequency, f) == KErrNone && f > 0) gFastFreq = f;
    TInt up = 1;
    if (HAL::Get(HAL::EFastCounterCountsUp, up) == KErrNone && !up) gFastDown = 1;
    TInt p = 0;
    if (HAL::Get(HAL::ENanoTickPeriod, p) == KErrNone && p > 0) gTickUs = p;
    Log(_L8("clock fastcounter=%d Hz down=%d ntick=%d us"), gFastFreq, gFastDown, gTickUs);
}
static TInt64 NowUs()
{
    if (gFastFreq > 0) {
        TInt64 c = (TInt64)(gFastDown ? (TUint32)(0 - User::FastCounter()) : User::FastCounter());   // 32 bit: wraps, tests are short so unwrap by delta
        static TUint32 last = 0;
        static TInt64 hi = 0;
        if ((TUint32)c < last) hi += (TInt64)1 << 32;
        last = (TUint32)c;
        return ((hi + (TInt64)last) * 1000000) / gFastFreq;
    }
    return (TInt64)User::NTickCount() * gTickUs;
}
static TInt NowMs() { return (TInt)(NowUs() / 1000); }

// ---- idle counter thread (lowest priority tight loop) ----
static volatile TUint32 gIdle = 0;
static TInt IdleFn(TAny*)
{
    for (;;) gIdle++;
    return 0;
}
static RThread gIdleThread;
static TInt gBaseRate = 0;     // idle increments per second with nothing running

// ---- waiting helper ----
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
    // Runs the scheduler until aMs elapsed, *aFlag != 0, or gAbort.
    void Run(TInt aMs, volatile TInt* aFlag)
    {
        iDeadline = aMs / 10;
        iTicks = 0;
        iFlag = aFlag;
        iTimer->Start(10000, 10000, TCallBack(Tick, this));
        iWait.Start();
        iTimer->Cancel();
    }
private:
    static TInt Tick(TAny* a)
    {
        CPump* s = (CPump*)a;
        if (++s->iTicks >= s->iDeadline || gAbort || (s->iFlag && *s->iFlag))
            s->iWait.AsyncStop();
        return 0;
    }
    CPeriodic* iTimer;
    CActiveSchedulerWait iWait;
    TInt iDeadline, iTicks;
    volatile TInt* iFlag;
};
static CPump* gPump;

// ---- per-test header/footer: cpu speed, ram, idle headroom ----
static TInt gT0Ms, gIdle0, gRam0;
static TInt RamFree() { TInt v = 0; HAL::Get(HAL::EMemoryRAMFree, v); return v; }
static void TestBegin(const TDesC8& aName)
{
    TInt cpu = 0;
    HAL::Get(HAL::ECPUSpeed, cpu);
    gRam0 = RamFree();
    Log(_L8("TEST BEGIN %S cpu=%d kHz ram_free=%d"), &aName, cpu, gRam0);
    gAbort = 0;
    gT0Ms = NowMs();
    gIdle0 = (TInt)gIdle;
}
static void TestEnd(const TDesC8& aName)
{
    TInt el = NowMs() - gT0Ms;
    TUint32 d = (TUint32)gIdle - (TUint32)gIdle0;
    TInt rate = el > 0 ? (TInt)(((TInt64)d * 1000) / el) : 0;
    TInt hr100 = gBaseRate > 0 ? (TInt)(((TInt64)rate * 10000) / gBaseRate) : -1;
    TInt ram = RamFree();
    Log(_L8("TEST END %S elapsed=%d ms idle_rate=%d/s baseline=%d/s headroom_x100=%d%% ram_free=%d (delta %d)"),
        &aName, el, rate, gBaseRate, hr100, ram, ram - gRam0);
}
// headroom of a sub-interval (used inside a test to split encode/decode)
struct THead { TInt iMs0; TUint32 iC0; };
static void HeadBegin(THead& h) { h.iMs0 = NowMs(); h.iC0 = gIdle; }
static TInt HeadEnd(const THead& h)   // returns headroom x100 (percent*100)
{
    TInt el = NowMs() - h.iMs0;
    TUint32 d = (TUint32)gIdle - h.iC0;
    TInt rate = el > 0 ? (TInt)(((TInt64)d * 1000) / el) : 0;
    return gBaseRate > 0 ? (TInt)(((TInt64)rate * 10000) / gBaseRate) : -1;
}

// ---- helpers ----
static TInt HexVal(TUint8 c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}
// "video/H264; profile-level-id=42800A" -> 0x42800A, -1 if none
static TInt ProfileLevel(const TDesC8& aMime)
{
    TInt p = aMime.Find(_L8("profile-level-id="));
    if (p < 0) return -1;
    p += 17;
    TInt v = 0;
    for (TInt i = 0; i < 6; i++) {
        if (p + i >= aMime.Length()) return -1;
        TInt h = HexVal(aMime[p + i]);
        if (h < 0) return -1;
        v = (v << 4) | h;
    }
    return v;
}
static TInt NeedLevel(TInt w, TInt h, TInt fps)
{
    static const TInt T[][3] = { { 0x0A, 99, 1485 }, { 0x0B, 396, 3000 }, { 0x0C, 396, 6000 },
                                 { 0x0D, 396, 11880 }, { 0x14, 396, 11880 }, { 0x15, 792, 19800 },
                                 { 0x1E, 1620, 40500 } };
    TInt mbs = ((w + 15) / 16) * ((h + 15) / 16);
    TInt mbps = mbs * fps;
    for (TUint i = 0; i < sizeof(T) / sizeof(T[0]); i++)
        if (mbs <= T[i][1] && mbps <= T[i][2]) return T[i][0];
    return 0x1E;
}
// Picks from a format list the baseline (4280xx) entry with the smallest level >= need.
static CCompressedVideoFormat* PickFormat(const RPointerArray<CCompressedVideoFormat>& aList, TInt aNeed)
{
    CCompressedVideoFormat* best = NULL;
    TInt bestLvl = 0x7fff;
    for (TInt i = 0; i < aList.Count(); i++) {
        TInt pl = ProfileLevel(aList[i]->MimeType());
        if (pl < 0 || (pl >> 8) != 0x4280) continue;
        TInt lvl = pl & 0xFF;
        if (lvl >= aNeed && lvl < bestLvl) { bestLvl = lvl; best = aList[i]; }
    }
    if (!best && aList.Count() > 0) best = aList[0];
    return best;
}
// NAL scan of an Annex B unit: sets flags for VCL (type 1/5), IDR (5), SPS (7), PPS (8)
struct TNal { TBool iVcl, iIdr, iSps, iPps, iStart; };
static void ScanNal(const TDesC8& d, TNal& n)
{
    n.iVcl = n.iIdr = n.iSps = n.iPps = n.iStart = EFalse;
    const TUint8* p = d.Ptr();
    TInt len = d.Length();
    for (TInt i = 0; i + 3 < len; i++) {
        if (p[i] == 0 && p[i + 1] == 0 && p[i + 2] == 1) {
            n.iStart = ETrue;
            TInt t = p[i + 3] & 0x1F;
            if (t == 1 || t == 5) n.iVcl = ETrue;
            if (t == 5) n.iIdr = ETrue;
            if (t == 7) n.iSps = ETrue;
            if (t == 8) n.iPps = ETrue;
            i += 3;
        }
    }
    if (!n.iStart) n.iVcl = ETrue;    // not Annex B: assume every unit is a picture
}

// ---- coded stream store ----
struct TUnit { TInt iOff, iLen; TInt64 iTsUs; TBool iVcl, iKey; };
class CStream : public CBase
{
public:
    RArray<TUnit> iUnits;
    HBufC8* iData;
    TInt iUsed;
    TInt iSps, iPps;
    static CStream* NewL()
    {
        CStream* s = new (ELeave) CStream;
        CleanupStack::PushL(s);
        s->iData = HBufC8::NewL(64 * 1024);
        CleanupStack::Pop();
        return s;
    }
    ~CStream() { iUnits.Close(); delete iData; }
    void Add(const TDesC8& aD, TInt64 aTs, const TNal& aN)
    {
        if (iUsed + aD.Length() > 3 * 1024 * 1024) return;       // cap
        if (iUsed + aD.Length() > iData->Des().MaxLength()) {
            TInt nm = iData->Des().MaxLength() * 2;
            while (nm < iUsed + aD.Length()) nm *= 2;
            HBufC8* nb = iData->ReAlloc(nm);
            if (!nb) return;
            iData = nb;
        }
        TUnit u;
        u.iOff = iUsed; u.iLen = aD.Length(); u.iTsUs = aTs; u.iVcl = aN.iVcl; u.iKey = aN.iIdr;
        iData->Des().SetLength(iUsed);
        iData->Des().Append(aD);
        iUsed += aD.Length();
        if (aN.iSps) iSps++;
        if (aN.iPps) iPps++;
        iUnits.Append(u);
    }
    TPtrC8 Unit(TInt i) const { return TPtrC8(iData->Ptr() + iUnits[i].iOff, iUnits[i].iLen); }
};

// ---- encoder session ----
struct TEncResult
{
    TInt iOffered, iWritten, iDropped, iPictures, iKeys, iBytes, iErrors;
    TInt iLatAvg, iLatMax, iDurMs, iSkipped, iProcessed;
};

// ---- encoder configuration (swept by ENCSWEEP; the winner is used by all later tests) ----
struct TEncCfg
{
    TInt iUid;          // 0 = first result of FindEncodersL
    TInt iCodec;        // 0 H.264, 1 MPEG-4, 2 H.263
    TInt iInSel;        // -1 auto planar 4:2:0 entry, >=0 list index, -2/-3/-4 hand built variants
    const char* iMime;  // NULL = pick from the encoder's list by level (H.264) or default mime
    TInt iDu, iEncap;
    TInt iBufOpt, iRand, iConst, iRealtime, iRc, iFind, iInitWaitMs;
};
static TEncCfg gCfg = { 0x20001c13, 0, -1, NULL, EDuCodedPicture, EDuElementaryStream, 1, 1, 1, 1, 1, 0, 10000 };
static TBool gWinner = EFalse;
static TPtrC8 MimeOf(TInt aCodec)
{
    if (aCodec == 1) return TPtrC8((const TUint8*)"video/mp4v-es");
    if (aCodec == 2) return TPtrC8((const TUint8*)"video/H263-2000");
    return TPtrC8((const TUint8*)"video/H264");
}

static const TInt KPool = 6;

class CEncSession : public CBase, public MMMFDevVideoRecordObserver
{
public:
    CMMFDevVideoRecord* iRec;
    THwDeviceId iDev;
    TSize iSize;
    TInt iFrameBytes;
    TReal iFps;
    TVideoPicture* iPic[KPool];
    TPtr8* iPtr[KPool];
    HBufC8* iBuf[KPool];
    TBool iBusy[KPool];
    volatile TInt iInitDone, iInitErr, iFatal;
    CStream* iStream;
    TEncResult iR;
    TInt64 iLatSum;
    TInt iT0Ms;
    TInt iFrameNo;
    TUint8* iTmpl;           // synthetic texture template
    TBool iStarted;

    static CEncSession* NewL()
    {
        CEncSession* s = new (ELeave) CEncSession;
        CleanupStack::PushL(s);
        s->iStream = CStream::NewL();
        CleanupStack::Pop();
        return s;
    }
    ~CEncSession()
    {
        delete iRec;
        for (TInt i = 0; i < KPool; i++) { delete iPic[i]; delete iPtr[i]; delete iBuf[i]; }
        delete iStream;
        delete[] iTmpl;
    }

    TInt iNewBufCalls, iRetCalls, iUnitsOut;
    TEncCfg iCfg;

    // Returns KErrNone or the leave code.
    TInt Start(TSize aSz, TInt aFps, TInt aBitrateK) { return StartCfg(gCfg, aSz, aFps, aBitrateK); }
    TInt StartCfg(const TEncCfg& aCfg, TSize aSz, TInt aFps, TInt aBitrateK)
    {
        iCfg = aCfg;
        TRAPD(err, StartL(aSz, aFps, aBitrateK));
        return err;
    }

    void StartL(TSize aSz, TInt aFps, TInt aBitrateK)
    {
        const TEncCfg& c = iCfg;
        iSize = aSz;
        iFps = aFps;
        iFrameBytes = aSz.iWidth * aSz.iHeight * 3 / 2;
        Mem::FillZ(&iR, sizeof(iR));
        iFatal = 0; iInitDone = 0; iInitErr = 0; iFrameNo = 0; iLatSum = 0;
        iNewBufCalls = iRetCalls = iUnitsOut = 0;
        for (TInt i = 0; i < KPool; i++) {
            iBuf[i] = HBufC8::NewL(iFrameBytes);
            iBuf[i]->Des().SetLength(iFrameBytes);
            iPtr[i] = new (ELeave) TPtr8(iBuf[i]->Des());
            iPic[i] = new (ELeave) TVideoPicture;
            Mem::FillZ(iPic[i], sizeof(TVideoPicture));
            iPic[i]->iData.iDataFormat = EYuvRawData;
            iPic[i]->iData.iDataSize = aSz;
            iPic[i]->iData.iRawData = iPtr[i];
            iBusy[i] = EFalse;
        }
        iTmpl = new (ELeave) TUint8[aSz.iWidth + 512];
        TInt seed = 12345;
        for (TInt i = 0; i < aSz.iWidth + 512; i++) {
            seed = seed * 1103515245 + 12345;
            iTmpl[i] = (TUint8)(((i & 63) * 3) + ((seed >> 16) & 15) + 20);
        }

        Log(_L8("step enc: CMMFDevVideoRecord::NewL"));
        iRec = CMMFDevVideoRecord::NewL(*this);
        TUid uid = TUid::Uid(c.iUid);
        if (c.iFind == 1 || c.iUid == 0) {
            RArray<TUid> u;
            CleanupClosePushL(u);
            Log(_L8("step enc: FindEncodersL %s"), c.iCodec == 1 ? "mp4v" : (c.iCodec == 2 ? "h263" : "h264"));
            TRAPD(fe, iRec->FindEncodersL(MimeOf(c.iCodec), 0, u, EFalse));
            Log(_L8("enc FindEncodersL err=%d count=%d first=0x%08x"), fe, u.Count(), u.Count() ? u[0].iUid : 0);
            if (c.iUid == 0) {
                if (!u.Count()) User::Leave(KErrNotFound);
                uid = u[0];
            }
            CleanupStack::PopAndDestroy(&u);
        } else if (c.iFind == 2) {
            RArray<TUid> u;
            CleanupClosePushL(u);
            Log(_L8("step enc: GetEncoderListL"));
            TRAPD(fe, iRec->GetEncoderListL(u));
            Log(_L8("enc GetEncoderListL err=%d count=%d"), fe, u.Count());
            CleanupStack::PopAndDestroy(&u);
        }
        Log(_L8("step enc: SelectEncoderL 0x%08x"), uid.iUid);
        iDev = iRec->SelectEncoderL(uid);
        Log(_L8("step enc: VideoEncoderInfoLC"));
        CVideoEncoderInfo* info = iRec->VideoEncoderInfoLC(uid);

        TUncompressedVideoFormat fmt;
        TBool found = EFalse;
        const RArray<TUncompressedVideoFormat>& inf = info->SupportedInputFormats();
        if (c.iInSel >= 0 && c.iInSel < inf.Count()) { fmt = inf[c.iInSel]; found = ETrue; }
        for (TInt i = 0; i < inf.Count() && !found && c.iInSel < 0; i++) {
            if (inf[i].iDataFormat == EYuvRawData && inf[i].iYuvFormat.iDataLayout == EYuvDataPlanar &&
                (inf[i].iYuvFormat.iPattern & (EYuv420Chroma1 | EYuv420Chroma2 | EYuv420Chroma3))) {
                fmt = inf[i]; found = ETrue;
            }
        }
        if (c.iInSel <= -2) {      // hand built: start from the first YUV entry, then override fields
            for (TInt i = 0; i < inf.Count(); i++)
                if (inf[i].iDataFormat == EYuvRawData) { fmt = inf[i]; found = ETrue; break; }
        }
        if (!found) {
            Mem::FillZ(&fmt, sizeof(fmt));
            fmt.iDataFormat = EYuvRawData;
            fmt.iYuvFormat.iAspectRatioNum = 1;
            fmt.iYuvFormat.iAspectRatioDenom = 1;
            fmt.iYuvFormat.iCoefficients = (TYuvCoefficients)0x01000001;
        }
        if (c.iInSel <= -2) {
            fmt.iDataFormat = EYuvRawData;
            fmt.iYuvFormat.iPattern = (c.iInSel == -3) ? EYuv420Chroma1 : EYuv420Chroma2;
            fmt.iYuvFormat.iDataLayout = EYuvDataPlanar;
            if (c.iInSel == -4) fmt.iYuvFormat.iCoefficients = (TYuvCoefficients)((TInt)fmt.iYuvFormat.iCoefficients + 1);
        }
        Log(_L8("enc input fmt: datafmt=0x%x coeff=%d pattern=%d layout=%d aspect=%u/%u (list %d, sel %d)"),
            (TInt)fmt.iDataFormat, (TInt)fmt.iYuvFormat.iCoefficients, (TInt)fmt.iYuvFormat.iPattern,
            (TInt)fmt.iYuvFormat.iDataLayout, fmt.iYuvFormat.iAspectRatioNum, fmt.iYuvFormat.iAspectRatioDenom,
            inf.Count(), c.iInSel);

        TInt need = NeedLevel(aSz.iWidth, aSz.iHeight, aFps);
        CCompressedVideoFormat* cf = NULL;
        if (c.iMime) {
            TPtrC8 mp((const TUint8*)c.iMime);
            cf = CCompressedVideoFormat::NewL(mp);
            CleanupStack::PushL(cf);
        } else if (c.iCodec == 0) {
            cf = PickFormat(info->SupportedOutputFormats(), need);
            if (!cf) User::Leave(KErrNotSupported);
        } else {
            cf = CCompressedVideoFormat::NewL(MimeOf(c.iCodec));
            CleanupStack::PushL(cf);
        }
        TBuf8<80> mm;
        mm.Copy(cf->MimeType().Left(78));
        Log(_L8("enc output mime=%S (need level 0x%02x) du=0x%x encap=0x%x"), &mm, need, c.iDu, c.iEncap);

        Log(_L8("step enc: SetInputFormatL %dx%d"), aSz.iWidth, aSz.iHeight);
        iRec->SetInputFormatL(iDev, fmt, aSz);
        Log(_L8("step enc: SetOutputFormatL"));
        iRec->SetOutputFormatL(iDev, *cf, (TVideoDataUnitType)c.iDu, (TVideoDataUnitEncapsulation)c.iEncap, EFalse);
        if (c.iMime || c.iCodec != 0) CleanupStack::PopAndDestroy(cf);
        CleanupStack::PopAndDestroy(info);
        info = NULL;
        Log(_L8("step enc: SetSourceMemoryL %d fps const=%d realtime=%d"), aFps, c.iConst, c.iRealtime);
        iRec->SetSourceMemoryL(iFps, c.iConst, c.iRealtime);

        if (c.iBufOpt) {
            TEncoderBufferOptions bo;
            Mem::FillZ(&bo, sizeof(bo));
            bo.iMaxPreEncoderBufferPictures = 2;
            bo.iHrdVbvSpec = EHrdVbvNone;
            bo.iMaxOutputBufferSize = 0x10000;
            bo.iMaxCodedPictureSize = 0xFFFFFFFFu;
            bo.iMaxCodedSegmentSize = 0xFFFFFFFFu;
            bo.iMinNumOutputBuffers = 4;
            Log(_L8("step enc: SetBufferOptionsL"));
            TRAPD(e1, iRec->SetBufferOptionsL(bo));
            Log(_L8("enc SetBufferOptionsL err=%d"), e1);
        }
        if (c.iRand) {
            Log(_L8("step enc: SetMinRandomAccessRate 0.5"));
            iRec->SetMinRandomAccessRate(0.5);
        }

        Log(_L8("step enc: Initialize"));
        iRec->Initialize();
        gPump->Run(c.iInitWaitMs, &iInitDone);
        Log(_L8("enc init done=%d err=%d fatal=%d newbuf=%d"), (TInt)iInitDone, iInitErr, (TInt)iFatal, iNewBufCalls);
        if (!iInitDone) User::Leave(KErrTimedOut);
        if (iInitErr) User::Leave(iInitErr);

        if (c.iRc) {
            TRateControlOptions ro;
            Mem::FillZ(&ro, sizeof(ro));
            ro.iControl = EBrControlStream;
            ro.iBitrate = aBitrateK * 1000;
            ro.iPictureQuality = 50;
            ro.iPictureRate = iFps;
            ro.iQualityTemporalTradeoff = 0.5;
            ro.iLatencyQualityTradeoff = 0.0;
            Log(_L8("step enc: SetRateControlOptions %d kbit/s"), aBitrateK);
            iRec->SetRateControlOptions(0, ro);
            Log(_L8("step enc: CommitL"));
            TRAPD(e2, iRec->CommitL());
            Log(_L8("enc CommitL err=%d"), e2);
        }
        Log(_L8("step enc: Start"));
        iRec->Start();
        iStarted = ETrue;
        iT0Ms = NowMs();
    }

    TInt64 TsNowUs() { return NowUs() - (TInt64)iT0Ms * 1000; }

    // aYuv NULL = synthetic frame. aTsUs = capture time (us since Start).
    TBool Offer(const TUint8* aYuv, TInt aYuvLen, TInt64 aTsUs)
    {
        iR.iOffered++;
        if (!iStarted || iFatal) { iR.iDropped++; return EFalse; }
        TInt k = -1;
        for (TInt i = 0; i < KPool; i++) if (!iBusy[i]) { k = i; break; }
        if (k < 0) { iR.iDropped++; return EFalse; }
        TUint8* d = (TUint8*)iBuf[k]->Ptr();
        if (aYuv) {
            TInt n = aYuvLen < iFrameBytes ? aYuvLen : iFrameBytes;
            Mem::Copy(d, aYuv, n);
        } else {
            Synth(d);
        }
        TVideoPicture* p = iPic[k];
        iPtr[k]->SetLength(iFrameBytes);
        p->iOptions = TVideoPicture::ETimestamp;
        p->iTimestamp = TTimeIntervalMicroSeconds(aTsUs);
        iBusy[k] = ETrue;
        TRAPD(err, iRec->WritePictureL(p));
        if (err) {
            iBusy[k] = EFalse;
            iR.iErrors++;
            if (iR.iErrors <= 3) Log(_L8("enc WritePictureL err=%d"), err);
            return EFalse;
        }
        iR.iWritten++;
        return ETrue;
    }

    void Synth(TUint8* aDst)
    {
        const TInt w = iSize.iWidth, h = iSize.iHeight;
        const TInt f = iFrameNo++;
        for (TInt y = 0; y < h; y++)
            Mem::Copy(aDst + y * w, iTmpl + ((f * 3 + y * 2) & 255), w);
        TUint8* u = aDst + w * h;
        TUint8* v = u + (w / 2) * (h / 2);
        Mem::Fill(u, (w / 2) * (h / 2), 128 + (f & 15));
        Mem::Fill(v, (w / 2) * (h / 2), 128 - (f & 15));
        // moving bright box
        TInt bx = (f * 4) % (w - 40), by = (f * 3) % (h - 40);
        for (TInt y = by; y < by + 32; y++) Mem::Fill(aDst + y * w + bx, 32, 235);
    }

    // finishes encoding, fills statistics
    void Finish()
    {
        iR.iDurMs = NowMs() - iT0Ms;
        Log(_L8("step enc: Stop"));
        if (iRec) {
            iRec->Stop();
            gAbort = 0;
            gPump->Run(300, NULL);
            CMMFDevVideoRecord::TPictureCounters pc;
            Mem::FillZ(&pc, sizeof(pc));
            iRec->GetPictureCounters(pc);
            iR.iSkipped = pc.iPicturesSkippedBufferOverflow + pc.iPicturesSkippedProcPower +
                          pc.iPicturesSkippedRateControl;
            iR.iProcessed = pc.iPicturesProcessed;
            Log(_L8("enc counters: input=%d processed=%d skip_overflow=%d skip_power=%d skip_ratectl=%d"),
                pc.iInputPictures, pc.iPicturesProcessed, pc.iPicturesSkippedBufferOverflow,
                pc.iPicturesSkippedProcPower, pc.iPicturesSkippedRateControl);
        }
        iStarted = EFalse;
        if (iR.iPictures > 0) iR.iLatAvg = (TInt)(iLatSum / iR.iPictures);
    }

    void Report(const TDesC8& aTag)
    {
        TInt dur = iR.iDurMs > 0 ? iR.iDurMs : 1;
        TInt fps100 = (TInt)((TInt64)iR.iPictures * 100000 / dur);
        TInt kbps10 = (TInt)((TInt64)iR.iBytes * 8 * 10 / dur);   // bytes*8/ms = kbit/s, x10
        Log(_L8("RESULT enc %S: offered=%d written=%d dropped=%d skipped=%d encoded=%d keyframes=%d bytes=%d "
                "dur=%d ms fps_x100=%d bitrate_kbps_x10=%d lat_avg=%d ms lat_max=%d ms errors=%d sps=%d pps=%d fatal=%d"),
            &aTag, iR.iOffered, iR.iWritten, iR.iDropped, iR.iSkipped, iR.iPictures, iR.iKeys, iR.iBytes,
            iR.iDurMs, fps100, kbps10, iR.iLatAvg, iR.iLatMax, iR.iErrors, iStream->iSps, iStream->iPps,
            (TInt)iFatal);
    }

    // MMMFDevVideoRecordObserver
    void MdvroReturnPicture(TVideoPicture* aPic)
    {
        iRetCalls++;
        for (TInt i = 0; i < KPool; i++) if (iPic[i] == aPic) iBusy[i] = EFalse;
    }
    void MdvroSupplementalInfoSent() {}
    void MdvroNewBuffers()
    {
        iNewBufCalls++;
        if (!iRec) return;
        TInt n = iRec->NumDataBuffers();
        while (n-- > 0) {
            TVideoOutputBuffer* b = NULL;
            TRAPD(err, b = iRec->NextBufferL());
            if (err || !b) { if (err) iR.iErrors++; break; }
            TInt64 now = TsNowUs();
            TInt64 ts = b->iCaptureTimestamp.Int64();
            TNal nal;
            ScanNal(b->iData, nal);
            iR.iBytes += b->iData.Length();
            iUnitsOut++;
            iStream->Add(b->iData, ts, nal);
            if (nal.iVcl) {
                iR.iPictures++;
                if (nal.iIdr) iR.iKeys++;
                TInt lat = (TInt)((now - ts) / 1000);
                iLatSum += lat;
                if (lat > iR.iLatMax) iR.iLatMax = lat;
            }
            iRec->ReturnBuffer(b);
        }
    }
    void MdvroFatalError(TInt aErr) { iFatal = aErr; gAbort = 1; Log(_L8("enc fatal error %d"), aErr); }
    void MdvroInitializeComplete(TInt aErr) { iInitErr = aErr; iInitDone = 1; }
    void MdvroStreamEnd() {}
};

// ---- decoder session ----
struct TDecResult { TInt iWritten, iDecoded, iErrors, iLatAvg, iLatMax, iDurMs, iFatal; };
struct TWriteRec { TInt64 iTs; TInt iMs; };

class CDecSession : public CBase, public MMMFDevVideoPlayObserver
{
public:
    CMMFDevVideoPlay* iPlay;
    THwDeviceId iDev;
    volatile TInt iInitDone, iInitErr, iFatal;
    TDecResult iR;
    TInt64 iLatSum;
    RArray<TWriteRec> iWrites;
    CStream* iSrc;
    TInt iNext;
    TBool iPaced;
    TInt iT0Ms, iLastOutMs, iFirstLogged, iVclWritten;
    volatile TInt iDone;
    CPeriodic* iTick;

    static CDecSession* NewL() { return new (ELeave) CDecSession; }
    ~CDecSession()
    {
        delete iTick;
        delete iPlay;
        iWrites.Close();
    }

    void SetupL(TSize aSz, TInt aFps)
    {
        Mem::FillZ(&iR, sizeof(iR));
        iInitDone = iInitErr = iFatal = 0;
        iLatSum = 0;
        Log(_L8("step dec: CMMFDevVideoPlay::NewL"));
        iPlay = CMMFDevVideoPlay::NewL(*this);
        Log(_L8("step dec: SelectDecoderL 0x10204bfc"));
        iDev = iPlay->SelectDecoderL(TUid::Uid(0x10204bfc));
        Log(_L8("step dec: VideoDecoderInfoLC"));
        CVideoDecoderInfo* info = iPlay->VideoDecoderInfoLC(TUid::Uid(0x10204bfc));
        TInt need = NeedLevel(aSz.iWidth, aSz.iHeight, aFps);
        CCompressedVideoFormat* cf = PickFormat(info->SupportedFormats(), need);
        if (!cf) User::Leave(KErrNotSupported);
        TBuf8<80> mm;
        mm.Copy(cf->MimeType().Left(78));
        Log(_L8("dec input mime=%S (need level 0x%02x)"), &mm, need);
        Log(_L8("step dec: SetInputFormatL"));
        iPlay->SetInputFormatL(iDev, *cf, EDuCodedPicture, EDuElementaryStream, ETrue);
        CleanupStack::PopAndDestroy(info);

        Log(_L8("step dec: GetOutputFormatListL"));
        RArray<TUncompressedVideoFormat> of;
        CleanupClosePushL(of);
        iPlay->GetOutputFormatListL(iDev, of);
        for (TInt i = 0; i < of.Count(); i++) {
            if (of[i].iDataFormat == EYuvRawData)
                Log(_L8("  dec output[%d] yuv coeff=%d pattern=%d layout=%d"), i, (TInt)of[i].iYuvFormat.iCoefficients,
                    (TInt)of[i].iYuvFormat.iPattern, (TInt)of[i].iYuvFormat.iDataLayout);
            else
                Log(_L8("  dec output[%d] datafmt=0x%x rgb=0x%x"), i, (TInt)of[i].iDataFormat, (TInt)of[i].iRgbFormat);
        }
        if (of.Count() > 0) {
            Log(_L8("step dec: SetOutputFormatL [0]"));
            iPlay->SetOutputFormatL(iDev, of[0]);
        }
        CleanupStack::PopAndDestroy(&of);
        Log(_L8("step dec: SetVideoDestScreenL(false)"));
        TRAPD(e1, iPlay->SetVideoDestScreenL(EFalse));
        Log(_L8("dec SetVideoDestScreenL err=%d"), e1);
        iPlay->SynchronizeDecoding(EFalse);
        CMMFDevVideoPlay::TBufferOptions bo;
        iPlay->GetBufferOptions(bo);
        Log(_L8("dec default buffer opts: predec=%d maxpost=%d maxinput=%d mininbufs=%d"), bo.iPreDecodeBufferSize,
            bo.iMaxPostDecodeBufferSize, bo.iMaxInputBufferSize, bo.iMinNumInputBuffers);
        if (bo.iMaxInputBufferSize < 0x10000) bo.iMaxInputBufferSize = 0x10000;
        if (bo.iMinNumInputBuffers < 4) bo.iMinNumInputBuffers = 4;
        Log(_L8("step dec: SetBufferOptionsL"));
        TRAPD(e2, iPlay->SetBufferOptionsL(bo));
        Log(_L8("dec SetBufferOptionsL err=%d"), e2);
        Log(_L8("step dec: Initialize"));
        iPlay->Initialize();
        gPump->Run(10000, &iInitDone);
        Log(_L8("dec init done=%d err=%d fatal=%d"), (TInt)iInitDone, iInitErr, (TInt)iFatal);
        if (!iInitDone) User::Leave(KErrTimedOut);
        if (iInitErr) User::Leave(iInitErr);
        Log(_L8("step dec: CommitL"));
        TRAPD(e3, iPlay->CommitL());
        Log(_L8("dec CommitL err=%d"), e3);
    }

    // Feeds aSrc into the decoder. aPaced: honour capture timestamps, else flat out.
    void RunL(CStream* aSrc, TBool aPaced, TInt aMaxMs)
    {
        iSrc = aSrc;
        iPaced = aPaced;
        iNext = 0;
        iDone = 0;
        iFirstLogged = 0;
        iWrites.Reset();
        iVclWritten = 0;
        iTick = CPeriodic::NewL(CActive::EPriorityStandard);
        Log(_L8("step dec: Start (units=%d paced=%d)"), aSrc->iUnits.Count(), (TInt)aPaced);
        iPlay->Start();
        iT0Ms = iLastOutMs = NowMs();
        iTick->Start(1000, 3000, TCallBack(TickCb, this));
        gPump->Run(aMaxMs, &iDone);
        iTick->Cancel();
        iR.iDurMs = (iLastOutMs > iT0Ms ? iLastOutMs : NowMs()) - iT0Ms;
        Log(_L8("step dec: Stop"));
        iPlay->Stop();
        if (iR.iDecoded > 0) iR.iLatAvg = (TInt)(iLatSum / iR.iDecoded);
        CMMFDevVideoPlay::TPictureCounters pc;
        Mem::FillZ(&pc, sizeof(pc));
        iPlay->GetPictureCounters(pc);
        Log(_L8("dec counters: total=%d decoded=%d displayed=%d skipped=%d"), pc.iTotalPictures, pc.iPicturesDecoded,
            pc.iPicturesDisplayed, pc.iPicturesSkipped);
        iR.iFatal = iFatal;
    }

    static TInt TickCb(TAny* a) { ((CDecSession*)a)->Tick(); return 0; }
    void Tick()
    {
        TInt now = NowMs();
        TInt64 elapsedUs = (TInt64)(now - iT0Ms) * 1000;
        while (iNext < iSrc->iUnits.Count()) {
            const TUnit& u = iSrc->iUnits[iNext];
            if (iPaced && u.iTsUs > elapsedUs) break;
            TPtrC8 d = iSrc->Unit(iNext);
            TVideoInputBuffer* b = NULL;
            TRAPD(err, b = iPlay->GetBufferL(d.Length()));
            if (err) { iR.iErrors++; if (iR.iErrors <= 3) Log(_L8("dec GetBufferL err=%d"), err); iNext++; continue; }
            if (!b) break;     // no free buffer yet
            b->iData.Copy(d);
            b->iOptions = TVideoInputBuffer::EPresentationTimestamp;
            b->iPresentationTimestamp = TTimeIntervalMicroSeconds(u.iTsUs);
            TWriteRec wr; wr.iTs = u.iTsUs; wr.iMs = NowMs();
            TRAPD(e2, iPlay->WriteCodedDataL(b));
            if (e2) { iR.iErrors++; if (iR.iErrors <= 3) Log(_L8("dec WriteCodedDataL err=%d"), e2); }
            else {
                iR.iWritten++;
                if (u.iVcl) { iWrites.Append(wr); iVclWritten++; }
            }
            iNext++;
        }
        if (iNext >= iSrc->iUnits.Count()) {
            if (!iFirstLogged) { iFirstLogged = 1; Log(_L8("dec: all units written (%d)"), iR.iWritten); iPlay->InputEnd(); }
            // finished when nothing came out for 700 ms or every picture is out
            if (now - iLastOutMs > 700 || iR.iDecoded >= iVclWritten) iDone = 1;
        }
        if (iFatal) iDone = 1;
    }

    void Report(const TDesC8& aTag)
    {
        TInt dur = iR.iDurMs > 0 ? iR.iDurMs : 1;
        Log(_L8("RESULT dec %S: written=%d decoded=%d dur=%d ms fps_x100=%d lat_avg=%d ms lat_max=%d ms errors=%d fatal=%d"),
            &aTag, iR.iWritten, iR.iDecoded, iR.iDurMs, (TInt)((TInt64)iR.iDecoded * 100000 / dur), iR.iLatAvg,
            iR.iLatMax, iR.iErrors, iR.iFatal);
    }

    // MMMFDevVideoPlayObserver
    void MdvpoNewBuffers() {}
    void MdvpoReturnPicture(TVideoPicture*) {}
    void MdvpoSupplementalInformation(const TDesC8&, const TTimeIntervalMicroSeconds&, const TPictureId&) {}
    void MdvpoPictureLoss() {}
    void MdvpoPictureLoss(const TArray<TPictureId>&) {}
    void MdvpoSliceLoss(TUint, TUint, const TPictureId&) {}
    void MdvpoReferencePictureSelection(const TDesC8&) {}
    void MdvpoTimedSnapshotComplete(TInt, TPictureData*, const TTimeIntervalMicroSeconds&, const TPictureId&) {}
    void MdvpoNewPictures()
    {
        for (;;) {
            TVideoPicture* p = NULL;
            TRAPD(err, p = iPlay->NextPictureL());
            if (err || !p) { if (err) iR.iErrors++; break; }
            TInt now = NowMs();
            iLastOutMs = now;
            iR.iDecoded++;
            if (iR.iDecoded == 1)
                Log(_L8("dec first picture: fmt=0x%x size=%dx%d opts=0x%x"), (TInt)p->iData.iDataFormat,
                    p->iData.iDataSize.iWidth, p->iData.iDataSize.iHeight, (TInt)p->iOptions);
            if (p->iOptions & TVideoPicture::ETimestamp) {
                TInt64 ts = p->iTimestamp.Int64();
                for (TInt i = 0; i < iWrites.Count(); i++) {
                    if (iWrites[i].iTs == ts) {
                        TInt lat = now - iWrites[i].iMs;
                        iLatSum += lat;
                        if (lat > iR.iLatMax) iR.iLatMax = lat;
                        iWrites.Remove(i);
                        break;
                    }
                }
            }
            iPlay->ReturnPicture(p);
        }
    }
    void MdvpoFatalError(TInt aErr) { iFatal = aErr; gAbort = 1; iDone = 1; Log(_L8("dec fatal error %d"), aErr); }
    void MdvpoInitComplete(TInt aErr) { iInitErr = aErr; iInitDone = 1; }
    void MdvpoStreamEnd() {}
};

// removing matched writes would break the 'all out' check: keep a separate counter
// (iWrites shrinks as pictures match; iDecoded >= remaining is therefore only a hint,
// the 700 ms quiet period is the real end condition).

// ---- camera source ----
class CCamSrc : public CBase, public MCameraObserver
{
public:
    volatile TInt iReserved, iPowered, iReserveErr, iPowerErr;
    CEncSession* iSink;
    TInt iFrames, iBadLen, iErrs, iLastErr, iFirstMs, iLastMs, iLen;
    TInt iExpected;
    void ReserveComplete(TInt aErr) { iReserveErr = aErr; iReserved = 1; }
    void PowerOnComplete(TInt aErr) { iPowerErr = aErr; iPowered = 1; }
    void ViewFinderFrameReady(CFbsBitmap&) {}
    void ImageReady(CFbsBitmap* aB, HBufC8* aD, TInt) { delete aB; delete aD; }
    void FrameBufferReady(MFrameBuffer* aB, TInt aErr)
    {
        if (aErr != KErrNone) { iErrs++; iLastErr = aErr; if (aB) aB->Release(); return; }
        TInt now = NowMs();
        if (iFrames == 0) iFirstMs = now;
        iLastMs = now;
        iFrames++;
        if (aB && iSink) {
            TDesC8* d = NULL;
            TRAPD(err, d = aB->DataL(0));
            if (err || !d) { iErrs++; iLastErr = err; }
            else {
                iLen = d->Length();
                if (iLen != iExpected) iBadLen++;
                iSink->Offer(d->Ptr(), d->Length(), iSink->TsNowUs());
            }
        }
        if (aB) aB->Release();
    }
};

// ---- tests ----
struct TCtxS { CEncSession* iEnc; };
static TInt gMaxOffers = 0x7fffffff;
static TInt gRunStart = 0;
static TBool OverBudget(TInt aSec) { return NowMs() - gRunStart > aSec * 1000; }
static TInt SynthTick(TAny* a)
{
    CEncSession* e = ((TCtxS*)a)->iEnc;
    if (e->iR.iOffered < gMaxOffers) e->Offer(NULL, 0, e->TsNowUs());
    return 0;
}

static void FmtTag(TBuf8<64>& t, const char* kind, TInt w, TInt h, TInt fps, TInt br)
{
    t.Zero();
    t.Append(TPtrC8((const TUint8*)kind));
    t.AppendFormat(_L8(" %dx%d@%d %dk"), w, h, fps, br);
}

// decodes aStream; aPaced = capture timing, else flat out
static void DecodeStreamL(CStream* aStream, TInt w, TInt h, TInt fps, TInt aMaxMs, TBool aPaced, const TDesC8& aTag)
{
    if (aStream->iUnits.Count() == 0) { Log(_L8("no coded units to decode")); return; }
    CDecSession* dec = CDecSession::NewL();
    CleanupStack::PushL(dec);
    TRAPD(e2, dec->SetupL(TSize(w, h), fps));
    Log(_L8("dec setup err=%d"), e2);
    if (!e2) {
        THead hd;
        HeadBegin(hd);
        TRAPD(e3, dec->RunL(aStream, aPaced, aMaxMs));
        Log(_L8("dec run err=%d"), e3);
        TInt hr = HeadEnd(hd);
        dec->Report(aTag);
        Log(_L8("RESULT cpu dec %S: headroom_x100=%d%%"), &aTag, hr);
    }
    CleanupStack::PopAndDestroy(dec);
}

// TEST 1 (encode) for one configuration, then TEST 2 (decode flat out)
static void SynthConfigL(TInt w, TInt h, TInt fps, TInt br)
{
    TBuf8<64> tag;
    FmtTag(tag, "synth", w, h, fps, br);
    CStream* stream = NULL;
    TestBegin(tag);
    {
        CEncSession* enc = CEncSession::NewL();
        CleanupStack::PushL(enc);
        TInt err = enc->Start(TSize(w, h), fps, br);
        Log(_L8("enc start err=%d"), err);
        if (err == KErrNone) {
            THead hd;
            HeadBegin(hd);
            CPeriodic* t = CPeriodic::NewL(CActive::EPriorityStandard);
            CleanupStack::PushL(t);
            TCtxS ctx = { enc };
            TInt interval = (TInt)(1000000 / fps);
            Log(_L8("step enc: feeding synthetic frames for 6 s every %d us"), interval);
            t->Start(interval, interval, TCallBack(SynthTick, &ctx));
            gPump->Run(6000, NULL);
            t->Cancel();
            TInt hr = HeadEnd(hd);
            CleanupStack::PopAndDestroy(t);
            enc->Finish();
            enc->Report(tag);
            Log(_L8("RESULT cpu enc %S: headroom_x100=%d%%"), &tag, hr);
            stream = enc->iStream;
            enc->iStream = NULL;
        }
        Log(_L8("step enc: destroy"));
        CleanupStack::PopAndDestroy(enc);
    }
    TestEnd(tag);
    if (stream) {
        CleanupStack::PushL(stream);
        TBuf8<64> dtag;
        FmtTag(dtag, "decode-of-synth", w, h, fps, br);
        TestBegin(dtag);
        DecodeStreamL(stream, w, h, fps, 15000, EFalse, dtag);
        TestEnd(dtag);
        CleanupStack::PopAndDestroy(stream);
    }
}

// TEST 3 for one camera size on an open, powered-on camera
static void CamConfigL(CCamera* aCam, CCamSrc* aObs, TInt aIdx, const TCameraInfo& aInfo, TInt w, TInt h, TInt br)
{
    TBuf8<64> tag;
    FmtTag(tag, "cam", w, h, 15, br);
    tag.AppendFormat(_L8(" cam%d"), aIdx);
    // locate size index and the rate closest to 15 fps
    TInt sIdx = -1;
    for (TInt s = 0; s < aInfo.iNumVideoFrameSizesSupported; s++) {
        TSize sz;
        aCam->EnumerateVideoFrameSizes(sz, s, CCamera::EFormatYUV420Planar);
        if (sz.iWidth == w && sz.iHeight == h) { sIdx = s; break; }
    }
    if (sIdx < 0) { Log(_L8("%S: size not offered by camera, skipped"), &tag); return; }
    TInt rIdx = 0; TReal32 bestD = 1000;
    for (TInt r = 0; r < aInfo.iNumVideoFrameRatesSupported; r++) {
        TReal32 rate = 0;
        aCam->EnumerateVideoFrameRates(rate, r, CCamera::EFormatYUV420Planar, sIdx);
        TReal32 d = rate > 15 ? rate - 15 : 15 - rate;
        if (rate > 0 && d < bestD) { bestD = d; rIdx = r; }
    }
    TReal32 chosen = 0;
    aCam->EnumerateVideoFrameRates(chosen, rIdx, CCamera::EFormatYUV420Planar, sIdx);
    Log(_L8("%S: camera size idx=%d rate idx=%d (%d/100 fps)"), &tag, sIdx, rIdx, (TInt)(chosen * 100));

    CStream* stream = NULL;
    TestBegin(tag);
    {
        CEncSession* enc = CEncSession::NewL();
        CleanupStack::PushL(enc);
        TInt err = enc->Start(TSize(w, h), 15, br);
        Log(_L8("enc start err=%d"), err);
        if (err == KErrNone) {
            aObs->iFrames = aObs->iBadLen = aObs->iErrs = aObs->iLastErr = 0;
            aObs->iExpected = w * h * 3 / 2;
            Log(_L8("step cam%d: PrepareVideoCaptureL"), aIdx);
            TRAPD(pe, aCam->PrepareVideoCaptureL(CCamera::EFormatYUV420Planar, sIdx, rIdx, 2, 1));
            Log(_L8("cam prepare err=%d"), pe);
            if (!pe) {
                THead hd;
                HeadBegin(hd);
                aObs->iSink = enc;
                enc->iT0Ms = NowMs();
                Log(_L8("step cam%d: StartVideoCapture (6 s)"), aIdx);
                aCam->StartVideoCapture();
                gPump->Run(6000, NULL);
                Log(_L8("step cam%d: StopVideoCapture"), aIdx);
                aCam->StopVideoCapture();
                aObs->iSink = NULL;
                TInt hr = HeadEnd(hd);
                enc->Finish();
                TInt span = aObs->iLastMs - aObs->iFirstMs;
                TInt camfps = span > 0 ? (TInt)((TInt64)(aObs->iFrames - 1) * 100000 / span) : 0;
                Log(_L8("RESULT cam %S: frames=%d span=%d ms camera_fps_x100=%d errs=%d lasterr=%d framelen=%d expected=%d badlen=%d"),
                    &tag, aObs->iFrames, span, camfps, aObs->iErrs, aObs->iLastErr, aObs->iLen, aObs->iExpected,
                    aObs->iBadLen);
                enc->Report(tag);
                Log(_L8("RESULT cpu cam+enc %S: headroom_x100=%d%%"), &tag, hr);
                stream = enc->iStream;
                enc->iStream = NULL;
            }
        }
        CleanupStack::PopAndDestroy(enc);
    }
    TestEnd(tag);
    if (stream) {
        CleanupStack::PushL(stream);
        TBuf8<64> dtag;
        FmtTag(dtag, "decode-of-cam", w, h, 15, br);
        dtag.AppendFormat(_L8(" cam%d"), aIdx);
        TestBegin(dtag);
        DecodeStreamL(stream, w, h, 15, 12000, ETrue, dtag);
        TestEnd(dtag);
        CleanupStack::PopAndDestroy(stream);
    }
}

static void CameraTestL(TInt aIdx, const TInt aSizes[][3], TInt aCount)
{
    CCamSrc* obs = new (ELeave) CCamSrc;
    CleanupStack::PushL(obs);
    Log(_L8("step camera %d NewL"), aIdx);
    CCamera* cam = CCamera::NewL(*obs, aIdx);
    CleanupStack::PushL(cam);
    TInt tries = 0;
    for (; tries < 3; tries++) {
        obs->iReserved = 0; obs->iReserveErr = 0;
        Log(_L8("step camera %d Reserve (try %d)"), aIdx, tries + 1);
        cam->Reserve();
        gPump->Run(5000, &obs->iReserved);
        Log(_L8("camera %d reserve done=%d err=%d"), aIdx, (TInt)obs->iReserved, obs->iReserveErr);
        if (obs->iReserved && obs->iReserveErr == KErrNone) break;
        if (obs->iReserveErr != KErrInUse) { tries = 99; break; }
        gPump->Run(500, NULL);
    }
    if (tries >= 3 || !obs->iReserved || obs->iReserveErr) {
        Log(_L8("camera %d: not available, skipping"), aIdx);
        CleanupStack::PopAndDestroy(2);
        return;
    }
    Log(_L8("step camera %d PowerOn"), aIdx);
    obs->iPowered = 0; obs->iPowerErr = 0;
    cam->PowerOn();
    gPump->Run(5000, &obs->iPowered);
    Log(_L8("camera %d poweron done=%d err=%d"), aIdx, (TInt)obs->iPowered, obs->iPowerErr);
    if (obs->iPowered && !obs->iPowerErr) {
        TCameraInfo ci;
        cam->CameraInfo(ci);
        for (TInt i = 0; i < aCount; i++) {
            if (OverBudget(230)) { Log(_L8("time budget used, skipping camera config")); break; }
            TRAPD(err, CamConfigL(cam, obs, aIdx, ci, aSizes[i][0], aSizes[i][1], aSizes[i][2]));
            Log(_L8("camera %d config %dx%d end err=%d"), aIdx, aSizes[i][0], aSizes[i][1], err);
            gAbort = 0;
        }
        cam->PowerOff();
    }
    Log(_L8("step camera %d Release"), aIdx);
    cam->Release();
    CleanupStack::PopAndDestroy(2);
}

// ---- ENCSWEEP: find an encoder configuration whose Initialize succeeds AND yields coded data ----
static TInt gInitOnly = 0;

static void LogFmt(const TDesC8& aTag, TInt i, const TUncompressedVideoFormat& f)
{
    if (f.iDataFormat == EYuvRawData)
        Log(_L8("  %S[%d] datafmt=0x%x(yuv) coeff=%d(0x%x) pattern=%d layout=%d aspect=%u/%u"), &aTag, i,
            (TInt)f.iDataFormat, (TInt)f.iYuvFormat.iCoefficients, (TInt)f.iYuvFormat.iCoefficients,
            (TInt)f.iYuvFormat.iPattern, (TInt)f.iYuvFormat.iDataLayout, f.iYuvFormat.iAspectRatioNum,
            f.iYuvFormat.iAspectRatioDenom);
    else
        Log(_L8("  %S[%d] datafmt=0x%x(rgb) rgbformat=0x%x"), &aTag, i, (TInt)f.iDataFormat, (TInt)f.iRgbFormat);
}

static TInt gInCount = 0;
static TUint8 gInIsYuv[32];
static TInt gOtherUids[4];
static TInt gOtherCount = 0;

static void SweepInfoL()
{
    CEncSession* dummy = NULL; (void)dummy;
    class CObs : public CBase, public MMMFDevVideoRecordObserver
    {
    public:
        void MdvroReturnPicture(TVideoPicture*) {}
        void MdvroSupplementalInfoSent() {}
        void MdvroNewBuffers() {}
        void MdvroFatalError(TInt) {}
        void MdvroInitializeComplete(TInt) {}
        void MdvroStreamEnd() {}
    };
    CObs* obs = new (ELeave) CObs;
    CleanupStack::PushL(obs);
    Log(_L8("step sweep info: NewL"));
    CMMFDevVideoRecord* rec = CMMFDevVideoRecord::NewL(*obs);
    CleanupStack::PushL(rec);
    for (TInt codec = 0; codec < 3; codec++) {
        RArray<TUid> u;
        CleanupClosePushL(u);
        TBuf8<40> m; m.Copy(MimeOf(codec));
        Log(_L8("step sweep info: FindEncodersL %S"), &m);
        TRAPD(fe, rec->FindEncodersL(MimeOf(codec), 0, u, EFalse));
        Log(_L8("sweep advertised: encoders for %S err=%d count=%d"), &m, fe, u.Count());
        for (TInt i = 0; i < u.Count(); i++) {
            Log(_L8("  encoder uid 0x%08x"), u[i].iUid);
            if (codec == 0 && u[i].iUid != 0x20001c13 && gOtherCount < 4) gOtherUids[gOtherCount++] = u[i].iUid;
        }
        CleanupStack::PopAndDestroy(&u);
    }
    Log(_L8("step sweep info: VideoEncoderInfoLC 0x20001c13"));
    CVideoEncoderInfo* d = rec->VideoEncoderInfoLC(TUid::Uid(0x20001c13));
    Log(_L8("sweep advertised: accelerated=%d directcapture=%d maxpic=%dx%d maxbitrate=%u layers=%u dutypes=0x%x encaps=0x%x"),
        (TInt)d->Accelerated(), (TInt)d->SupportsDirectCapture(), d->MaxPictureSize().iWidth, d->MaxPictureSize().iHeight,
        d->MaxBitrate(), d->MaxBitrateLayers(), (TInt)d->SupportedDataUnitTypes(), (TInt)d->SupportedDataUnitEncapsulations());
    Log(_L8("sweep advertised: pictureoptions=0x%x uep=%u scal=%u pictureloss=%d sliceloss=%d suppl=%d"),
        (TInt)d->SupportedPictureOptions(), d->MaxUnequalErrorProtectionLevels(), d->MaxInLayerScalabilitySteps(),
        (TInt)d->SupportsPictureLoss(), (TInt)d->SupportsSliceLoss(), (TInt)d->SupportsSupplementalEnhancementInfo());
    for (TInt i = 0; i < d->MaxPictureRates().Count(); i++)
        Log(_L8("  advertised max rate %dx%d @ %d/100 fps"), d->MaxPictureRates()[i].iPictureSize.iWidth,
            d->MaxPictureRates()[i].iPictureSize.iHeight, (TInt)(d->MaxPictureRates()[i].iPictureRate * 100));
    gInCount = d->SupportedInputFormats().Count();
    TBuf8<8> tg(_L8("input"));
    for (TInt i = 0; i < gInCount; i++) {
        LogFmt(tg, i, d->SupportedInputFormats()[i]);
        if (i < 32) gInIsYuv[i] = d->SupportedInputFormats()[i].iDataFormat == EYuvRawData;
    }
    for (TInt i = 0; i < d->SupportedOutputFormats().Count(); i++) {
        TBuf8<80> m; m.Copy(d->SupportedOutputFormats()[i]->MimeType().Left(78));
        Log(_L8("  advertised output mime[%d]=%S"), i, &m);
    }
    CleanupStack::PopAndDestroy(3);
}

// 1 = init ok and at least one coded unit came out
static TInt TryVariant(const TEncCfg& aC, const char* aName)
{
    TBuf8<72> n;
    n.Copy(TPtrC8((const TUint8*)aName));
    TInt t0 = NowMs();
    Log(_L8("sweep %S: begin"), &n);
    CEncSession* e = CEncSession::NewL();
    CleanupStack::PushL(e);
    gAbort = 0;
    TEncCfg c = aC;
    c.iInitWaitMs = 1500;
    TInt err = e->StartCfg(c, TSize(176, 144), 15, 128);
    TInt got = 0;
    if (err == KErrNone && gInitOnly) {
        Log(_L8("sweep %S: ok (init only, not a YUV input)"), &n);
    } else if (err == KErrNone) {
        CPeriodic* t = CPeriodic::NewL(CActive::EPriorityStandard);
        CleanupStack::PushL(t);
        TCtxS ctx = { e };
        gMaxOffers = 10;
        t->Start(66666, 66666, TCallBack(SynthTick, &ctx));
        gPump->Run(1800, NULL);
        t->Cancel();
        gMaxOffers = 0x7fffffff;
        CleanupStack::PopAndDestroy(t);
        got = e->iUnitsOut;
        Log(_L8("sweep %S: ok init; offered=%d written=%d werr=%d units=%d newbuf=%d retpic=%d fatal=%d"), &n,
            e->iR.iOffered, e->iR.iWritten, e->iR.iErrors, got, e->iNewBufCalls, e->iRetCalls, (TInt)e->iFatal);
        e->Finish();
    } else {
        Log(_L8("sweep %S: err=%d init_cb_err=%d done=%d fatal=%d newbuf=%d"), &n, err, e->iInitErr, (TInt)e->iInitDone,
            (TInt)e->iFatal, e->iNewBufCalls);
    }
    CleanupStack::PopAndDestroy(e);
    Log(_L8("sweep %S: took %d ms %s"), &n, NowMs() - t0, got ? "DATA" : "no data");
    return got > 0;
}

static void LogCfg(const TDesC8& aTag, const TEncCfg& c)
{
    TBuf8<80> m;
    if (c.iMime) m.Copy(TPtrC8((const TUint8*)c.iMime)); else m.Copy(_L8("(auto from list)"));
    Log(_L8("sweep %S uid=0x%08x codec=%d insel=%d mime=%S du=0x%x encap=0x%x bufopts=%d randaccess=%d const=%d realtime=%d ratectl_after_init=%d find=%d"),
        &aTag, c.iUid, c.iCodec, c.iInSel, &m, c.iDu, c.iEncap, c.iBufOpt, c.iRand, c.iConst, c.iRealtime, c.iRc, c.iFind);
}

static TBool gWon;
static TEncCfg gWinCfg;
static TBool Try(const TEncCfg& c, const char* aName, TInt aBudgetS = 60)
{
    if (gWon) return ETrue;
    if (NowMs() - gRunStart > aBudgetS * 1000) { Log(_L8("sweep budget exhausted, skipping %s"), aName); return EFalse; }
    TInt r = 0;
    TRAPD(le, r = TryVariant(c, aName));
    if (le) Log(_L8("sweep %s: leave %d"), aName, le);
    if (r) {
        gWon = ETrue; gWinCfg = c;
        TBuf8<72> n; n.Copy(TPtrC8((const TUint8*)aName));
        LogCfg(_L8("WINNER"), c);
        Log(_L8("sweep WINNER %S"), &n);
    }
    return gWon;
}

static void EncSweepL()
{
    gWon = EFalse;
    TRAPD(ie, SweepInfoL());
    Log(_L8("sweep info end err=%d"), ie);
    TEncCfg b = gCfg;
    Try(b, "base(first yuv420 planar entry, mime from list, coded/elementary)");
    // (1) input formats
    for (TInt i = 0; i < gInCount && i < 32; i++) {
        TEncCfg v = b; v.iInSel = i;
        TBuf8<110> nm; nm.Format(_L8("input[%d]"), i); nm.ZeroTerminate();
        gInitOnly = gInIsYuv[i] ? 0 : 1;
        Try(v, (const char*)nm.PtrZ());
        gInitOnly = 0;
    }
    { TEncCfg v = b; v.iInSel = -2; Try(v, "input hand chroma2 planar"); }
    { TEncCfg v = b; v.iInSel = -3; Try(v, "input hand chroma1 planar"); }
    { TEncCfg v = b; v.iInSel = -4; Try(v, "input hand coeff+1 chroma2"); }
    // (2) mimes
    static const char* const KM[] = { "video/H264", "video/H264; profile-level-id=42800A", "video/H264; profile-level-id=42800B",
        "video/H264; profile-level-id=42800C", "video/H264; profile-level-id=42900B", "video/H264; profile-level-id=428014", 0 };
    for (TInt i = 0; KM[i]; i++) { TEncCfg v = b; v.iMime = KM[i]; Try(v, KM[i]); }
    // (3) data unit / encapsulation
    static const TInt KDu[2] = { EDuCodedPicture, EDuVideoSegment };
    static const TInt KEn[2] = { EDuElementaryStream, EDuGenericPayload };
    for (TInt i = 0; i < 2; i++) for (TInt j = 0; j < 2; j++) {
        if (i == 0 && j == 0) continue;
        TEncCfg v = b; v.iDu = KDu[i]; v.iEncap = KEn[j];
        TBuf8<110> nm; nm.Format(_L8("du=%d encap=%d"), i, j); nm.ZeroTerminate();
        Try(v, (const char*)nm.PtrZ());
    }
    // (4) buffer options / random access / source memory flags
    { TEncCfg v = b; v.iBufOpt = 0; Try(v, "no SetBufferOptionsL"); }
    { TEncCfg v = b; v.iRand = 0; Try(v, "no SetMinRandomAccessRate"); }
    { TEncCfg v = b; v.iBufOpt = 0; v.iRand = 0; Try(v, "neither bufopts nor randaccess"); }
    for (TInt i = 0; i < 3; i++) {
        TEncCfg v = b; v.iConst = (i == 0 || i == 2) ? 0 : 1; v.iRealtime = (i == 0 || i == 1) ? 0 : 1;
        if (i == 2) { v.iConst = 0; v.iRealtime = 1; }
        TBuf8<110> nm; nm.Format(_L8("source memory const=%d realtime=%d"), v.iConst, v.iRealtime); nm.ZeroTerminate();
        Try(v, (const char*)nm.PtrZ());
    }
    // (5) rate control: only the documented order (after Initialize) or not at all; a call before Initialize
    // would violate the header precondition (panic), so it is not tried.
    { TEncCfg v = b; v.iRc = 0; Try(v, "no SetRateControlOptions"); }
    // (6) selection variants
    { TEncCfg v = b; v.iFind = 1; Try(v, "FindEncodersL before Select"); }
    { TEncCfg v = b; v.iFind = 2; Try(v, "GetEncoderListL before Select"); }
    { TEncCfg v = b; v.iUid = 0; Try(v, "uid from FindEncodersL first result"); }
    for (TInt i = 0; i < gOtherCount; i++) {
        TEncCfg v = b; v.iUid = gOtherUids[i]; TBuf8<110> nm; nm.Format(_L8("other H264 encoder 0x%08x"), gOtherUids[i]); nm.ZeroTerminate();
        Try(v, (const char*)nm.PtrZ());
    }
    // combined minimal setups
    TEncCfg m = b; m.iBufOpt = 0; m.iRand = 0; m.iConst = 0; m.iRealtime = 0;
    Try(m, "minimal (no bufopts/randaccess, src const=0 rt=0)");
    for (TInt i = 0; KM[i]; i++) { TEncCfg v = m; v.iMime = KM[i]; TBuf8<110> nm; nm.Format(_L8("minimal + %s"), KM[i]); nm.ZeroTerminate(); Try(v, (const char*)nm.PtrZ()); }
    for (TInt i = 0; i < 2; i++) for (TInt j = 0; j < 2; j++) {
        if (i == 0 && j == 0) continue;
        TEncCfg v = m; v.iDu = KDu[i]; v.iEncap = KEn[j]; TBuf8<110> nm; nm.Format(_L8("minimal du=%d encap=%d"), i, j); nm.ZeroTerminate();
        Try(v, (const char*)nm.PtrZ());
    }
    { TEncCfg v = m; v.iInSel = -2; Try(v, "minimal + hand chroma2"); }
    { TEncCfg v = m; v.iInSel = -3; Try(v, "minimal + hand chroma1"); }
    { TEncCfg v = m; v.iInSel = -4; Try(v, "minimal + hand coeff+1"); }
    // (7) other codecs, only to learn whether ANY encoder produces data
    if (!gWon) {
        Log(_L8("sweep alt-codec: H.264 found nothing, trying MPEG-4 and H.263"));
        TEncCfg a = m; a.iMime = NULL; a.iInSel = -1;
        TEncCfg v1 = a; v1.iCodec = 1; v1.iUid = 0x101f86d5;
        TEncCfg v2 = a; v2.iCodec = 1; v2.iUid = 0;
        TEncCfg v3 = a; v3.iCodec = 2; v3.iUid = 0;
        TInt base = gRunStart;
        gRunStart = NowMs();               // give the alt-codecs their own 15 s window
        Try(v1, "alt-codec mpeg4 uid 0x101f86d5", 15);
        if (gWon) { gWon = EFalse; Log(_L8("sweep alt-codec mpeg4(0x101f86d5) PRODUCES DATA")); } 
        Try(v2, "alt-codec mpeg4 via FindEncodersL", 15);
        if (gWon) { gWon = EFalse; Log(_L8("sweep alt-codec mpeg4(find) PRODUCES DATA")); }
        Try(v3, "alt-codec h263 via FindEncodersL", 15);
        if (gWon) { gWon = EFalse; Log(_L8("sweep alt-codec h263 PRODUCES DATA")); }
        gRunStart = base;
        Log(_L8("sweep NO WINNER for H.264"));
    } else {
        gCfg = gWinCfg;
        gWinner = ETrue;
        LogCfg(_L8("WINNER used by all later tests:"), gCfg);
    }
}

// ---- DECODER-ONLY: embedded known-good baseline clip, no encoder involved ----
static void DecoderOnlyL()
{
    const TInt len = sizeof(KClip);
    // find NAL start offsets (include a leading zero byte for 4 byte start codes)
    RArray<TInt> st;
    CleanupClosePushL(st);
    for (TInt i = 0; i + 3 < len; i++) {
        if (KClip[i] == 0 && KClip[i + 1] == 0 && KClip[i + 2] == 1) {
            st.Append((i > 0 && KClip[i - 1] == 0) ? i - 1 : i);
            i += 2;
        }
    }
    Log(_L8("clip: %d bytes, %d NALs"), len, st.Count());
    // access units: close a unit at each VCL NAL
    RArray<TInt> au;   // pairs start,end
    CleanupClosePushL(au);
    TInt accStart = 0;
    for (TInt k = 0; k < st.Count(); k++) {
        TInt s0 = st[k];
        TInt e0 = (k + 1 < st.Count()) ? st[k + 1] : len;
        TInt p = s0; while (p < len && KClip[p] == 0) p++;
        TInt type = KClip[p + 1] & 0x1F;       // p points at the 01 of the start code
        if (type == 1 || type == 5) { au.Append(accStart); au.Append(e0); accStart = e0; }
    }
    Log(_L8("clip: %d access units"), au.Count() / 2);
    CStream* all = CStream::NewL();
    CleanupStack::PushL(all);
    CStream* few = CStream::NewL();
    CleanupStack::PushL(few);
    TInt n = au.Count() / 2;
    for (TInt loop = 0; loop < 100 && n > 0; loop++) {
        for (TInt k = 0; k < n; k++) {
            TPtrC8 u(KClip + au[2 * k], au[2 * k + 1] - au[2 * k]);
            TNal nal; ScanNal(u, nal);
            TInt64 ts = (TInt64)(loop * n + k) * 66666;
            all->Add(u, ts, nal);
            if (loop < 15) few->Add(u, ts, nal);
        }
    }
    TBuf8<64> tag(_L8("decoder-only clip 176x144 flat-out"));
    TestBegin(tag);
    DecodeStreamL(all, 176, 144, 30, 30000, EFalse, tag);
    TestEnd(tag);
    TBuf8<64> tag2(_L8("decoder-only clip 176x144 paced 15fps"));
    TestBegin(tag2);
    DecodeStreamL(few, 176, 144, 30, 12000, ETrue, tag2);
    TestEnd(tag2);
    CleanupStack::PopAndDestroy(4);
}

// ---- main ----
static void MeasureBaselineL()
{
    Log(_L8("step baseline (2 s idle)"));
    TUint32 c0 = gIdle;
    TInt t0 = NowMs();
    gPump->Run(2000, NULL);
    TInt el = NowMs() - t0;
    TUint32 d = (TUint32)gIdle - c0;
    gBaseRate = el > 0 ? (TInt)(((TInt64)d * 1000) / el) : 0;
    Log(_L8("baseline idle_rate=%d/s over %d ms"), gBaseRate, el);
}

static void Step(const char* aName, void (*aFn)())
{
    Log(_L8("=== step %s begin"), aName);
    TRAPD(err, aFn());
    Log(_L8("=== step %s end err=%d"), aName, err);
}

static void Test12L()
{
    if (!gWinner) { Log(_L8("test12 skipped: no working encoder configuration")); return; }
    static const TInt KCfg[4][3] = { { 176, 144, 15 }, { 320, 240, 15 }, { 352, 288, 15 }, { 176, 144, 30 } };
    static const TInt KBr[3] = { 256, 128, 384 };
    for (TInt b = 0; b < 3; b++) {
        for (TInt c = 0; c < 4; c++) {
            if (OverBudget(215)) { Log(_L8("time budget used, skipping rest of test12")); return; }
            TRAPD(err, SynthConfigL(KCfg[c][0], KCfg[c][1], KCfg[c][2], KBr[b]));
            Log(_L8("synth config %dx%d@%d %dk end err=%d"), KCfg[c][0], KCfg[c][1], KCfg[c][2], KBr[b], err);
            gAbort = 0;
        }
    }
}
static void Test3L()
{
    if (!gWinner) { Log(_L8("test3 skipped: no working encoder configuration")); return; }
    static const TInt K0[2][3] = { { 176, 144, 128 }, { 320, 240, 256 } };
    static const TInt K1[3][3] = { { 176, 144, 128 }, { 320, 240, 256 }, { 352, 288, 384 } };
    TRAPD(e0, CameraTestL(0, K0, 2));
    Log(_L8("camera 0 test end err=%d"), e0);
    TRAPD(e1, CameraTestL(1, K1, 3));
    Log(_L8("camera 1 test end err=%d"), e1);
}

static void RunL()
{
    User::LeaveIfError(gFs.Connect());
    _LIT(KCard, "E:\\retrotele_videobench.txt");
    _LIT(KPhone, "C:\\Data\\retrotele_videobench.txt");
    gFs.MkDirAll(KPhone);
    if (gFile.Replace(gFs, KCard, EFileWrite | EFileShareAny) != KErrNone)
        User::LeaveIfError(gFile.Replace(gFs, KPhone, EFileWrite | EFileShareAny));
    gOpen = ETrue;
    Log(_L8("bench start"));
    CActiveScheduler* sch = new (ELeave) CActiveScheduler;
    CleanupStack::PushL(sch);
    CActiveScheduler::Install(sch);
    gPump = CPump::NewL();
    CleanupStack::PushL(gPump);
    InitClock();
    Log(_L8("step idle thread"));
    TInt e = gIdleThread.Create(_L("vbidle"), IdleFn, 0x1000, NULL, NULL);
    Log(_L8("idle thread create err=%d"), e);
    if (e == KErrNone) {
        gIdleThread.SetPriority(EPriorityMuchLess);
        gIdleThread.Resume();
    }
    TInt cpu = 0;
    HAL::Get(HAL::ECPUSpeed, cpu);
    Log(_L8("cpu=%d kHz ram_free=%d"), cpu, RamFree());
    Step("baseline", MeasureBaselineL);
    gRunStart = NowMs();
    Step("encsweep", EncSweepL);
    Step("decoder_only", DecoderOnlyL);
    Step("test12", Test12L);
    Step("test3", Test3L);
    TUint32 c0 = gIdle; TInt t0 = NowMs();
    gPump->Run(1000, NULL);
    Log(_L8("final idle_rate=%d/s (baseline %d/s)"), (TInt)(((TInt64)((TUint32)gIdle - c0) * 1000) / (NowMs() - t0)), gBaseRate);
    if (e == KErrNone) { gIdleThread.Kill(0); gIdleThread.Close(); }
    CleanupStack::PopAndDestroy(2);
}

GLDEF_C TInt E32Main()
{
    CTrapCleanup* cleanup = CTrapCleanup::New();
    TRAPD(err, RunL());
    if (err && gOpen) Log(_L8("fatal err=%d"), err);
    if (gOpen) { Log(_L8("bench end")); gFile.Close(); }
    gFs.Close();
    delete cleanup;
    return err;
}
