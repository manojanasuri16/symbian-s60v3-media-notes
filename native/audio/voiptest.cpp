/*
 * tgvoiptest.exe: full-duplex mic->speaker echo test using Nokia's VoIP
 * Audio Services (VAS). MMF streams mute the mic while playing; VAS doesn't.
 * Command line = seconds (default 20, max 120). Writes one stats line per
 * second to E:\retrotele_audiotest_voip.txt (or C:\Data\ with no card).
 * PCM16 is assumed to be 8 kHz mono (16-bit LE) on this device.
 */
#include <e32base.h>
#include <e32std.h>
#include <f32file.h>
#include <VoIPUtilityFactory.h>
#include <VoIPDownlinkStream.h>
#include <VoIPUplinkStream.h>
#include <VoIPDataBuffer.h>
#include <VoIPFormatIntfc.h>

const TInt KRate = 8000;                 // assumed PCM16 sample rate
const TInt KDelayBytes = KRate * 2 / 2;  // 500 ms
const TInt KBeepBytes = KRate * 2 / 2;   // 0.5 s beep
const TInt KFifoSize = 32768;
// 500 Hz at 8 kHz = 16 samples per cycle
static const TInt16 KBeep[16] = {
    0, 3061, 5657, 7391, 8000, 7391, 5657, 3061,
    0, -3061, -5657, -7391, -8000, -7391, -5657, -3061 };

class CRunner : public CBase, public MVoIPDownlinkObserver,
                public MVoIPUplinkObserver, public MVoIPFormatObserver
{
public:
    CRunner(TInt aSeconds) : iSeconds(aSeconds) {}
    ~CRunner()
    {
        delete iTimer;
        if (iUp) { iUp->Stop(); iUp->Close(); }
        if (iDn) { iDn->Stop(); iDn->Close(); }
        delete iUpFmt;
        delete iDnFmt;
        delete iUp;
        delete iDn;
        delete iPlayBuf;
        delete iFactory;
        iFormats.Close();
        iFile.Close();
        iFs.Close();
    }
    void StartL()
    {
        User::LeaveIfError(iFs.Connect());
        _LIT(KCard, "E:\\retrotele_audiotest_voip.txt");
        _LIT(KPhone, "C:\\Data\\retrotele_audiotest_voip.txt");
        if (iFile.Replace(iFs, KCard, EFileWrite | EFileShareAny) != KErrNone)
            User::LeaveIfError(iFile.Replace(iFs, KPhone, EFileWrite | EFileShareAny));
        Write(_L8("tgvoiptest start\n"));
        iTimer = CPeriodic::NewL(CActive::EPriorityStandard);
        iTimer->Start(1000000, 1000000, TCallBack(Tick, this));
        Setup();
    }
    // Any setup failure is logged; the tick loop still runs and ends the test.
    TBool Check(const TDesC8& aStep, TInt aErr)
    {
        if (aErr != KErrNone) {
            iErrors++;
            iLast = aErr;
            if (!iFirstErr) {
                iFirstErr = aErr;
                TBuf8<96> b;
                b.Format(_L8("first_error %S %d\n"), &aStep, aErr);
                Write(b);
            }
            if (iState == 0) iState = 3;
            return EFalse;
        }
        return ETrue;
    }
    void Setup()
    {
        if (!Check(_L8("CreateFactory"), CVoIPUtilityFactory::CreateFactory(iFactory))) return;
        if (!Check(_L8("CreateDownlink"), iFactory->CreateDownlinkStream(TVersion(1, 0, 0),
                   CVoIPUtilityFactory::EVoIPCall, iDn))) return;
        if (!Check(_L8("CreateUplink"), iFactory->CreateUplinkStream(TVersion(1, 0, 0),
                   CVoIPUtilityFactory::EVoIPCall, iUp))) return;
        if (!Check(_L8("CreateBuffer"), iFactory->CreateBuffer(iPlayBuf,
                   CVoIPDataBuffer::EStandard))) return;
        LogFormats(_L8("down"), ETrue);
        LogFormats(_L8("up"), EFalse);
        if (!Check(_L8("DnSetFormat"), iDn->SetFormat(EPCM16, iDnFmt))) return;
        Check(_L8("DnFmtObserver"), iDnFmt->SetObserver(*this));
        if (!Check(_L8("UpSetFormat"), iUp->SetFormat(EPCM16, iUpFmt))) return;
        Check(_L8("UpFmtObserver"), iUpFmt->SetObserver(*this));
        if (!Check(_L8("DnOpen"), iDn->Open(*this))) return;
        Check(_L8("UpOpen"), iUp->Open(*this));
    }
    void LogFormats(const TDesC8& aWhich, TBool aDown)
    {
        iFormats.Reset();
        TInt err = aDown ? iFactory->GetSupportedDownlinkFormats(iFormats)
                         : iFactory->GetSupportedUplinkFormats(iFormats);
        TBuf8<128> b;
        b.Format(_L8("formats_%S err %d:"), &aWhich, err);
        for (TInt i = 0; i < iFormats.Count() && b.Length() < 100; i++) {
            b.AppendFormat(_L8(" %d"), (TInt)iFormats[i]);
        }
        b.Append(_L8(" (1=AMRNB 2=G711 3=G729 4=ILBC 5=PCM16)\n"));
        Write(b);
    }
    void MaybeStart()
    {
        if (!iDnOpen || !iUpOpen || iState != 0) return;
        TInt mv = 0, mg = 0;
        Check(_L8("SetAudioDevice"), iDn->SetAudioDevice(CVoIPAudioDownlinkStream::ELoudSpeaker));
        iDn->GetMaxVolume(mv);
        Check(_L8("SetVolume"), iDn->SetVolume(mv * 70 / 100));
        iUp->GetMaxGain(mg);
        Check(_L8("SetGain"), iUp->SetGain(mg)   /* N79 max gain is 1: 3/4 rounded to 0 = muted */);
        TInt g = -1;
        iUp->GetGain(g);
        TBuf8<64> gm;
        gm.Format(_L8("maxgain %d gain %d maxvol %d\n"), mg, g, mv);
        Write(gm);
        iBeepLeft = KBeepBytes;
        if (!Check(_L8("DnStart"), iDn->Start())) return;
        if (!Check(_L8("UpStart"), iUp->Start())) return;
        iState = 1;
    }
    // MVoIPDownlinkObserver
    void FillBuffer(const CVoIPAudioDownlinkStream&, CVoIPDataBuffer* aBuffer)
    {
        TPtr8 p(NULL, 0);
        aBuffer->GetPayloadPtr(p);
        if (iDownBufs == 0) {
            TBuf8<64> m;
            m.Format(_L8("down_maxlen %d len %d\n"), p.MaxLength(), p.Length());
            Write(m);
        }
        TInt n = p.MaxLength() < 320 ? (p.MaxLength() & ~1) : 320; // one 20 ms frame
        p.SetLength(n);
        TUint8* d = (TUint8*)p.Ptr();
        for (TInt i = 0; i < n; i += 2) {
            TInt16 s = 0;
            if (iBeepLeft > 0) {
                s = KBeep[iBeepPos++ & 15];
                iBeepLeft -= 2;
            } else if (iPlaying || iFifoLen >= KDelayBytes) {
                if (iFifoLen >= 2) {
                    iPlaying = ETrue;
                    TUint8 lo = iFifo[iRd], hi = iFifo[(iRd + 1) % KFifoSize];
                    iRd = (iRd + 2) % KFifoSize;
                    iFifoLen -= 2;
                    s = (TInt16)(lo | (hi << 8));
                } else {
                    iPlaying = EFalse; // underrun: rebuild the delay
                }
            }
            d[i] = (TUint8)(s & 0xFF);
            d[i + 1] = (TUint8)((s >> 8) & 0xFF);
            TInt a = s < 0 ? -s : s;
            if (a > iPeakOut) iPeakOut = a;
        }
        iPlayBuf->SetPayloadPtr(p);
        iDownBytes += n;
        iDownBufs++;
        iDownTick++;
        TInt err = iDn->BufferFilled(iPlayBuf);
        if (err != KErrNone) { iErrors++; iLast = err; }
    }
    void Event(const CVoIPAudioDownlinkStream&, TInt aType, TInt aErr)
    {
        if (aType == MVoIPDownlinkObserver::KOpenComplete) {
            if (aErr == KErrNone) iDnOpen = ETrue; else Check(_L8("DnOpenComplete"), aErr);
            MaybeStart();
        } else if (aType == MVoIPDownlinkObserver::KDownlinkError) {
            Check(_L8("DnError"), aErr ? aErr : -1);
        }
    }
    // MVoIPUplinkObserver
    void EmptyBuffer(const CVoIPAudioUplinkStream&, CVoIPDataBuffer* aBuffer)
    {
        TPtr8 p(NULL, 0);
        aBuffer->GetPayloadPtr(p);
        TInt n = p.Length() & ~1;
        const TUint8* s = p.Ptr();
        for (TInt i = 0; i < n; i += 2) {
            TInt16 v = (TInt16)(s[i] | (s[i + 1] << 8));
            TInt a = v < 0 ? -v : v;
            if (a > iPeakIn) iPeakIn = a;
            if (iFifoLen + 2 <= KFifoSize) {
                iFifo[iWr] = s[i];
                iFifo[(iWr + 1) % KFifoSize] = s[i + 1];
                iWr = (iWr + 2) % KFifoSize;
                iFifoLen += 2;
            }
        }
        iUpBytes += n;
        iUpBufs++;
        if (iUpBufs == 10) {
            TBuf8<128> m;
            m.Format(_L8("up10 len %d max %d hex"), p.Length(), p.MaxLength());
            for (TInt k = 0; k < 16 && k < p.Length(); k++) m.AppendFormat(_L8(" %02x"), s[k]);
            m.Append(_L8("\n"));
            Write(m);
        }
        TInt err = iUp->BufferEmptied(aBuffer);
        if (err != KErrNone) { iErrors++; iLast = err; }
    }
    void Event(const CVoIPAudioUplinkStream&, TInt aType, TInt aErr)
    {
        if (aType == MVoIPUplinkObserver::KOpenComplete) {
            if (aErr == KErrNone) iUpOpen = ETrue; else Check(_L8("UpOpenComplete"), aErr);
            MaybeStart();
        } else if (aType == MVoIPUplinkObserver::KUplinkError) {
            Check(_L8("UpError"), aErr ? aErr : -1);
        }
    }
    // MVoIPFormatObserver
    void Event(const CVoIPFormatIntfc&, TInt) {}

    static TInt Tick(TAny* aSelf)
    {
        ((CRunner*)aSelf)->DoTick();
        return 0;
    }
    void DoTick()
    {
        iTicks++;
        TBuf8<256> line;
        line.Format(_L8("t %d state %d up_bytes %d down_bytes %d up_bufs %d down_bufs %d "
                        "errors %d last %d in %d out %d fmt %d\n"),
                    iTicks, iState, iUpBytes, iDownBytes, iUpBufs, iDownTick,
                    iErrors, iLast, iPeakIn, iPeakOut, (TInt)EPCM16);
        iPeakIn = 0;
        iPeakOut = 0;
        iDownTick = 0;
        Write(line);
        if (iTicks >= iSeconds) {
            iTimer->Cancel();
            Write(_L8("tgvoiptest end\n"));
            CActiveScheduler::Stop();
        }
    }
    void Write(const TDesC8& aText)
    {
        iFile.Write(aText);
        iFile.Flush();
    }
private:
    TInt iSeconds;
    TInt iTicks;
    RFs iFs;
    RFile iFile;
    CPeriodic* iTimer;
    CVoIPUtilityFactory* iFactory;
    CVoIPAudioDownlinkStream* iDn;
    CVoIPAudioUplinkStream* iUp;
    CVoIPFormatIntfc* iDnFmt;
    CVoIPFormatIntfc* iUpFmt;
    CVoIPDataBuffer* iPlayBuf;
    RArray<TVoIPCodecFormat> iFormats;
    TBool iDnOpen, iUpOpen, iPlaying;
    TInt iState, iErrors, iLast, iFirstErr;
    TInt iUpBytes, iDownBytes, iUpBufs, iDownBufs;
    TInt iPeakIn, iPeakOut;
    TInt iDownTick;
    TInt iBeepLeft, iBeepPos;
    TUint8 iFifo[KFifoSize];
    TInt iRd, iWr, iFifoLen;
};

static void RunL()
{
    TInt seconds = 20;
    TBuf<16> cmd;
    if (User::CommandLineLength() <= cmd.MaxLength()) {
        User::CommandLine(cmd);
        TLex lex(cmd);
        TInt v;
        if (lex.Val(v) == KErrNone && v > 0 && v <= 120)
            seconds = v;
    }
    CActiveScheduler* sched = new (ELeave) CActiveScheduler;
    CleanupStack::PushL(sched);
    CActiveScheduler::Install(sched);
    CRunner* r = new (ELeave) CRunner(seconds);
    CleanupStack::PushL(r);
    r->StartL();
    CActiveScheduler::Start();
    CleanupStack::PopAndDestroy(2, sched);
}

GLDEF_C TInt E32Main()
{
    CTrapCleanup* cleanup = CTrapCleanup::New();
    TRAPD(err, RunL());
    delete cleanup;
    return err;
}
