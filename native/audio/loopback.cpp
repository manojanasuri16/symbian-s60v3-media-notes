/* loopback.cpp: the echo engine (see loopback.h). */
#include "loopback.h"

/* one period of a 500 Hz tone at 16 kHz (32 samples), for the start beep */
static const TInt16 KBeep[32] = {0, 1756, 3444, 5000, 6364, 7483, 8315, 8827, 9000, 8827, 8315, 7483, 6364, 5000, 3444, 1756, 0, -1756, -3444, -5000, -6364, -7483, -8315, -8827, -9000, -8827, -8315, -7483, -6364, -5000, -3444, -1756};

static TInt RateCap(TInt aRate)
{
    switch (aRate) {
    case 8000: return TMdaAudioDataSettings::ESampleRate8000Hz;
    case 11025: return TMdaAudioDataSettings::ESampleRate11025Hz;
    case 12000: return TMdaAudioDataSettings::ESampleRate12000Hz;
    case 22050: return TMdaAudioDataSettings::ESampleRate22050Hz;
    case 24000: return TMdaAudioDataSettings::ESampleRate24000Hz;
    case 32000: return TMdaAudioDataSettings::ESampleRate32000Hz;
    case 44100: return TMdaAudioDataSettings::ESampleRate44100Hz;
    case 48000: return TMdaAudioDataSettings::ESampleRate48000Hz;
    default: return TMdaAudioDataSettings::ESampleRate16000Hz;
    }
}

CLoopback::CLoopback(TInt aRate, TInt aDelayMs, TInt aVolume)
    : iRate(aRate), iVolume(aVolume)
{
    iFrameBytes = aRate * 2 * KFrameMs / 1000;
    iDelayBytes = aRate * 2 * aDelayMs / 1000;
    if (iDelayBytes > KMaxFifo / 2)
        iDelayBytes = KMaxFifo / 2;
}

CLoopback* CLoopback::NewL(TInt aRate, TInt aDelayMs, TInt aVolume)
{
    CLoopback* self = new (ELeave) CLoopback(aRate, aDelayMs, aVolume);
    CleanupStack::PushL(self);
    self->ConstructL();
    CleanupStack::Pop(self);
    return self;
}

void CLoopback::ConstructL()
{
    iFifo = (TUint8*)User::AllocL(KMaxFifo);
    iBeepLeft = iRate / 2;          /* 0.5 s beep before the echo starts */
    iStart.HomeTime();
    iFirstRecordMs = -1;
    iFirstPlayMs = -1;

    /* The speaker is opened first and kept busy (silence) before the
       microphone starts: with the microphone open first, S60's audio policy
       rejected every write with KErrAccessDenied (-21) on the N79. */
    iIn = CMdaAudioInputStream::NewL(*this);
    iInSettings.Query();
    iInSettings.iSampleRate = RateCap(iRate);
    iInSettings.iChannels = TMdaAudioDataSettings::EChannelsMono;
    iInSettings.iFlags = TMdaAudioDataSettings::ENoNetworkRouting;
    iIn->SetPriority(EMdaPriorityMax, EMdaPriorityPreferenceTimeAndQuality);

    iOut = CMdaAudioOutputStream::NewL(*this);
    iOut->SetPriority(EMdaPriorityMax, EMdaPriorityPreferenceTimeAndQuality);
    iOutSettings.Query();
    iOutSettings.iSampleRate = RateCap(iRate);
    iOutSettings.iChannels = TMdaAudioDataSettings::EChannelsMono;
    iOutSettings.iFlags = TMdaAudioDataSettings::ENoNetworkRouting;
    iOutSettings.iVolume = 0;
    iOut->Open(&iOutSettings);
}

CLoopback::~CLoopback()
{
    iStopping = ETrue;
    if (iIn) {
        iIn->Stop();
        delete iIn;
    }
    delete iRoute;
    if (iOut) {
        iOut->Stop();
        delete iOut;
    }
    User::Free(iFifo);
}

TInt CLoopback::ElapsedMs() const
{
    TTime now;
    now.HomeTime();
    return (TInt)(now.MicroSecondsFrom(iStart).Int64() / 1000);
}

void CLoopback::Stop()
{
    iStopping = ETrue;
    if (iIn)
        iIn->Stop();
    if (iOut)
        iOut->Stop();
    iState = 2;
}

void CLoopback::Read()
{
    if (iStopping)
        return;
    iReadBuf.SetLength(0);
    TRAPD(err, iIn->ReadL(iReadBuf));
    if (err != KErrNone) {
        iReadErrors++;
        iLastError = err;
    }
}

void CLoopback::MaiscOpenComplete(TInt aError)
{
    if (aError != KErrNone) {
        iLastError = aError;
        iState = 3;
        return;
    }
    TRAPD(err, iIn->SetAudioPropertiesL(RateCap(iRate),
                                        TMdaAudioDataSettings::EChannelsMono));
    if (err != KErrNone)
        iLastError = err;
    iIn->SetGain(iIn->MaxGain());
    iState = 1;
    Read();
}

void CLoopback::MaiscBufferCopied(TInt aError, const TDesC8& aBuffer)
{
    if (aError != KErrNone) {
        if (aError != KErrAbort) {
            iReadErrors++;
            iLastError = aError;
        }
        return;
    }
    if (iFirstRecordMs < 0)
        iFirstRecordMs = ElapsedMs();
    iRecordBuffers++;
    TInt n = aBuffer.Length();
    iRecorded += n;
    /* peak level of this buffer (16-bit little-endian samples) */
    const TUint8* p = aBuffer.Ptr();
    for (TInt i = 0; i + 1 < n; i += 2) {
        TInt s = (TInt16)(p[i] | (p[i + 1] << 8));
        if (s < 0) s = -s;
        if (s > iMaxLevel) iMaxLevel = s;
        if (s > iPeakIn) iPeakIn = s;
    }
    /* append to the FIFO, dropping the oldest audio if it's full */
    if (n > KMaxFifo) {
        p += n - KMaxFifo;
        n = KMaxFifo;
    }
    if (iFifoLen + n > KMaxFifo) {
        TInt drop = iFifoLen + n - KMaxFifo;
        Mem::Move(iFifo, iFifo + drop, iFifoLen - drop);
        iFifoLen -= drop;
    }
    Mem::Copy(iFifo + iFifoLen, p, n);
    iFifoLen += n;
    Read();
    if (iOutOpen && !iWriting)
        WriteNext();
}

void CLoopback::MaiscRecordComplete(TInt aError)
{
    if (aError != KErrNone && aError != KErrCancel && aError != KErrAbort)
        iLastError = aError;
}

void CLoopback::WriteNext()
{
    if (iStopping || !iOutOpen || iWriting)
        return;
    TInt n = iFrameBytes * 2;        /* 40 ms per write */
    if (iPlaying || iFifoLen >= iDelayBytes)
        iPlaying = ETrue;
    if (n > iWriteBuf.MaxLength())
        n = iWriteBuf.MaxLength();
    if (iPlaying && iFifoLen >= n) {
        iWriteBuf.Copy(iFifo, n);
        Mem::Move(iFifo, iFifo + n, iFifoLen - n);
        iFifoLen -= n;
    } else if (!iPlaying && iBeepLeft > 0) {
        /* a short beep first: proves the speaker is audible */
        iWriteBuf.SetLength(n);
        TUint8* w = (TUint8*)iWriteBuf.Ptr();
        for (TInt i = 0; i + 1 < n; i += 2) {
            TInt16 v = 0;
            if (iBeepLeft > 0) {
                v = KBeep[iBeepPos & 31];
                iBeepPos++;
                iBeepLeft--;
            }
            w[i] = (TUint8)(v & 0xff);
            w[i + 1] = (TUint8)((v >> 8) & 0xff);
        }
    } else {
        /* building the delay or starving: play silence so the output
           stream keeps running */
        iWriteBuf.SetLength(n);
        iWriteBuf.FillZ();
        if (iPlaying)
            iSilence++;
    }
    {
        const TUint8* q = iWriteBuf.Ptr();
        for (TInt i = 0; i + 1 < iWriteBuf.Length(); i += 2) {
            TInt s = (TInt16)(q[i] | (q[i + 1] << 8));
            if (s < 0) s = -s;
            if (s > iPeakOut) iPeakOut = s;
        }
    }
    TRAPD(err, iOut->WriteL(iWriteBuf));
    if (err != KErrNone) {
        iWriteErrors++;
        iLastError = err;
        return;
    }
    iWriting = ETrue;
}

void CLoopback::MaoscOpenComplete(TInt aError)
{
    if (aError != KErrNone) {
        iLastError = aError;
        iState = 3;
        return;
    }
    TRAPD(err, iOut->SetAudioPropertiesL(RateCap(iRate),
                                         TMdaAudioDataSettings::EChannelsMono));
    if (err != KErrNone)
        iLastError = err;
    iOut->SetVolume(iOut->MaxVolume() * iVolume / 100);
    if (!iRoute) {
        /* play through the loudspeaker, not the earpiece */
        TRAPD(rerr, iRoute = CAudioOutput::NewL(*iOut));
        if (rerr == KErrNone && iRoute) {
            TRAP(rerr, iRoute->SetAudioOutputL(CAudioOutput::EPublic));
        }
        iRouted = (rerr == KErrNone) ? 1 : rerr;
    }
    iOutOpen = ETrue;
    WriteNext();
    if (!iInOpened) {
        iInOpened = ETrue;
        iIn->Open(&iInSettings);     /* microphone after the speaker */
    }
}

void CLoopback::MaoscBufferCopied(TInt aError, const TDesC8& aBuffer)
{
    iWriting = EFalse;
    if (aError != KErrNone) {
        if (aError != KErrAbort) {
            iWriteErrors++;
            iLastError = aError;
        }
        return;
    }
    if (iFirstPlayMs < 0)
        iFirstPlayMs = ElapsedMs();
    iPlayed += aBuffer.Length();
    WriteNext();
}

void CLoopback::MaoscPlayComplete(TInt aError)
{
    /* KErrUnderflow when we ran dry; restart writing if still running */
    iWriting = EFalse;
    if (aError != KErrNone && aError != KErrCancel && aError != KErrAbort) {
        iLastError = aError;
        if (!iStopping && aError == KErrUnderflow) {
            iOutOpen = EFalse;
            iOut->Open(&iOutSettings);   /* reopen and continue */
        }
    }
}

