/* loopback.h: microphone -> FIFO -> speaker echo engine (shared by
   tgcall.pyd and tgaudiotest.exe). */
#ifndef TG_LOOPBACK_H
#define TG_LOOPBACK_H
#include <e32base.h>
#include <e32std.h>
#include <mdaaudioinputstream.h>
#include <mdaaudiooutputstream.h>
#include <mda/common/audio.h>
#include <audiooutput.h>

const TInt KFrameMs = 20;
const TInt KMaxFifo = 64000;        /* bytes: 2 s at 16 kHz mono */

class CLoopback : public CBase,
                  public MMdaAudioInputStreamCallback,
                  public MMdaAudioOutputStreamCallback
{
public:
    static CLoopback* NewL(TInt aRate, TInt aDelayMs, TInt aVolume);
    ~CLoopback();
    void Stop();

    /* statistics */
    TInt iState;             /* 0 opening, 1 running, 2 stopped, 3 failed */
    TInt iRecorded;
    TInt iPlayed;
    TInt iRecordBuffers;
    TInt iSilence;
    TInt iReadErrors;
    TInt iWriteErrors;
    TInt iLastError;
    TInt iFirstRecordMs;
    TInt iFirstPlayMs;
    TInt iMaxLevel;
    TInt iPeakIn;            /* loudest sample since the last stats read */
    TInt iPeakOut;
    TInt iRouted;            /* 1 loudspeaker forced, <0 error */

    /* MMdaAudioInputStreamCallback */
    void MaiscOpenComplete(TInt aError);
    void MaiscBufferCopied(TInt aError, const TDesC8& aBuffer);
    void MaiscRecordComplete(TInt aError);
    /* MMdaAudioOutputStreamCallback */
    void MaoscOpenComplete(TInt aError);
    void MaoscBufferCopied(TInt aError, const TDesC8& aBuffer);
    void MaoscPlayComplete(TInt aError);

    TInt FifoBytes() const { return iFifoLen; }

private:
    CLoopback(TInt aRate, TInt aDelayMs, TInt aVolume);
    void ConstructL();
    void Read();
    void WriteNext();
    TInt ElapsedMs() const;

    CMdaAudioInputStream* iIn;
    CMdaAudioOutputStream* iOut;
    TMdaAudioDataSettings iInSettings;
    TMdaAudioDataSettings iOutSettings;
    TBuf8<4096> iReadBuf;
    TBuf8<4096> iWriteBuf;
    TUint8* iFifo;
    TInt iFifoLen;
    TInt iRate;
    TInt iDelayBytes;
    TInt iFrameBytes;
    TInt iVolume;
    TBool iOutOpen;
    TBool iWriting;
    TBool iPlaying;          /* delay reached once */
    TBool iStopping;
    TBool iInOpened;
    TInt iBeepLeft;          /* beep samples still to play at start */
    TInt iBeepPos;
    CAudioOutput* iRoute;
    TTime iStart;
};

#endif
