/*
 * tgaudiotest.exe: the echo test in its own process with MultimediaDD.
 *
 * RetroTele itself can't hold MultimediaDD (Python for S60's DLLs don't have
 * it), and without it S60's audio policy may refuse to play while recording.
 * This helper runs the same loopback for N seconds (command line, default
 * 20) and writes one stats line per second to E:\retrotele_audiotest.txt
 * (or C:\Data\ when there's no memory card). Needs a phone that installs
 * self-signed packages with device capabilities.
 */
#include <e32base.h>
#include <e32std.h>
#include <f32file.h>
#include "loopback.h"

class CRunner : public CBase
{
public:
    CRunner(TInt aSeconds) : iSeconds(aSeconds) {}
    ~CRunner()
    {
        delete iTimer;
        delete iLoop;
        iFile.Close();
        iFs.Close();
    }
    void StartL()
    {
        User::LeaveIfError(iFs.Connect());
        _LIT(KCard, "E:\\retrotele_audiotest.txt");
        _LIT(KPhone, "C:\\Data\\retrotele_audiotest.txt");
        if (iFile.Replace(iFs, KCard, EFileWrite | EFileShareAny) != KErrNone)
            User::LeaveIfError(iFile.Replace(iFs, KPhone, EFileWrite | EFileShareAny));
        Write(_L8("tgaudiotest start\n"));
        iLoop = CLoopback::NewL(16000, 500, 70);
        iTimer = CPeriodic::NewL(CActive::EPriorityStandard);
        iTimer->Start(1000000, 1000000, TCallBack(Tick, this));
    }
    static TInt Tick(TAny* aSelf)
    {
        ((CRunner*)aSelf)->DoTick();
        return 0;
    }
    void DoTick()
    {
        iTicks++;
        CLoopback* e = iLoop;
        TBuf8<256> line;
        line.Format(_L8("t %d state %d rec %d play %d bufs %d gaps %d rerr %d werr %d last %d "
                        "first_rec %d first_play %d fifo %d peak %d in %d out %d route %d\n"),
                    iTicks, e->iState, e->iRecorded, e->iPlayed, e->iRecordBuffers,
                    e->iSilence, e->iReadErrors, e->iWriteErrors, e->iLastError,
                    e->iFirstRecordMs, e->iFirstPlayMs, e->FifoBytes(), e->iMaxLevel,
                    e->iPeakIn, e->iPeakOut, e->iRouted);
        e->iPeakIn = 0;
        e->iPeakOut = 0;
        Write(line);
        if (iTicks >= iSeconds) {
            iTimer->Cancel();
            iLoop->Stop();
            Write(_L8("tgaudiotest end\n"));
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
    CLoopback* iLoop;
    CPeriodic* iTimer;
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
