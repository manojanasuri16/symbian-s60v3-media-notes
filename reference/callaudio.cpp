/*
 * tgcallaudio.exe: audio engine for Telegram calls. VoIP Audio Services
 * (PCM16, 8 kHz mono, 20 ms = 160 samples = 320 bytes) <-> Opus <-> UDP on
 * localhost. Uplink packets go to 127.0.0.1:47001, packets to play arrive on
 * 127.0.0.1:47002; a lone 'Q' datagram ends the program. Command line =
 * seconds (default 60, max 3600). Stats: E:\retrotele_callaudio.txt.
 * Opus runs on the main thread (exe has a 64 KB stack).
 *
 * RELAY MODE ("relay SECONDS"): the media goes straight to the Telegram
 * relays with the call protocol core (native/callproto, docs/CALL-PROTOCOL.md).
 * Parameters come from C:\Data\retrotele_call.cfg (E:\ as fallback), which
 * is deleted right after reading (it holds the key):
 *   out=0|1
 *   key=<512 hex chars>
 *   ep=<ipv4>,<port>,<32 hex chars peer_tag>      (up to 4 lines)
 * Python gets "S connecting|connected|reconnecting|failed|ended" datagrams on
 * 127.0.0.1:47001; a lone 'Q' to 127.0.0.1:47002 ends the program.
 *
 * VIDEO SEND (2.7.7 only, docs/CALL-VIDEO-SPEC.md): Python sends "V1"/"V0" (video on/off) to
 * 47002. tgvideo.exe (camera + H.264 encoder, other process, needs MultimediaDD) sends each
 * encoder unit as one UDP datagram 'V'+Annex-B to 127.0.0.1:47014. UDP on localhost was chosen
 * over a shared RChunk: units arrive as events in this active scheduler (no polling timer,
 * no seqlock, no extra thread), a unit is at most ~10 KB at 15 fps / 60 kbit/s (<= 40 KB/s),
 * and the rest of the helper chain (commands, status) already uses localhost UDP. All
 * per-packet work (RFC 6184 packetizing, RTP, RTCP SR, AES-CTR) stays native in c277_video.c /
 * c277_session.c. We send 'K' (IDR request) to tgvideo's 127.0.0.1:47012 on PLI/FIR and on
 * the early schedule. Info datagrams to Python: "P n" peer video state (0 off 1 paused
 * 2 active), "X n" peer asked for video n times, "A 0|1" our video announced active.
 */
#include <e32base.h>
#include <e32std.h>
#include <f32file.h>
#include <es_sock.h>
#include <in_sock.h>
#include <es_enum.h>
#include <VoIPUtilityFactory.h>
#include <VoIPDownlinkStream.h>
#include <VoIPUplinkStream.h>
#include <VoIPDataBuffer.h>
#include <VoIPFormatIntfc.h>
#include <libc/stdlib.h>
#include "opus.h"
#include <commdbconnpref.h>
extern "C" {
#include "callproto.h"
#include "c277_session.h"
}

const TInt KFrameBytes = 320;
const TInt KFrameSamples = 160;
const TInt KMaxPkt = 2048;
const TInt KRingSize = 30;
const TInt KPrefill = 6;       /* 120 ms: the RetroTele bridge delivers in bursts */
const TInt KPortOut = 47001;
const TInt KPortIn = 47002;
const TInt KMaxEp = 4;
const TInt KSends = 32;       /* an IDR is a burst of ~8 packets: keep slots free for audio */
const TInt KJbSlots = 12;
const TInt KJbPkt = 320;
const TInt KRelayFrame = 480;   /* 60 ms @ 8 kHz */
const TInt KPingMax = 6;
const TInt KMaxConn = 8;
const TInt KDnRing = 4800;      /* v277 downlink PCM ring, samples (600 ms) */
const TInt KDnPrefill = 960;    /* 120 ms */
const TInt KOpusMax = 1500;
const TInt KPortVideoIn = 47014;    /* tgvideo.exe -> here: 'V' + one Annex-B encoder unit */
const TInt KPortVideoCmd = 47012;   /* tgvideo.exe command port: 'K' = IDR, 'Q' = quit */
const TInt KPortVideoRx = 47015;    /* here -> tgvideo.exe: peer video, see PumpRx */
const TInt KVideoMax = 16400;       /* largest unit datagram (c277 C277_VAU_MAX + 'V') */

static TInt NowUs()
{
    TTime t;
    t.HomeTime();
    return (TInt)(t.Int64() % 2000000000);
}

static TUint32 NowMs()
{
    TTime t;
    t.HomeTime();
    return (TUint32)(t.Int64() / 1000);
}

static TInt HexVal(TUint8 c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

/* n bytes = 2n hex chars; EFalse on bad input */
static TBool ParseHex(const TUint8* aIn, TInt aChars, TUint8* aOut)
{
    if (aChars & 1) return EFalse;
    for (TInt i = 0; i < aChars / 2; i++) {
        TInt a = HexVal(aIn[2 * i]), b = HexVal(aIn[2 * i + 1]);
        if (a < 0 || b < 0) return EFalse;
        aOut[i] = (TUint8)(a * 16 + b);
    }
    return ETrue;
}

static TBool ParseInt(const TUint8* aIn, TInt aLen, TInt& aVal)
{
    if (aLen <= 0 || aLen > 9) return EFalse;
    aVal = 0;
    for (TInt i = 0; i < aLen; i++) {
        if (aIn[i] < '0' || aIn[i] > '9') return EFalse;
        aVal = aVal * 10 + (aIn[i] - '0');
    }
    return ETrue;
}

static TBool ParseIp(const TUint8* aIn, TInt aLen, TUint32& aIp)
{
    TInt part = 0, parts = 0, digits = 0;
    aIp = 0;
    for (TInt i = 0; i <= aLen; i++) {
        if (i == aLen || aIn[i] == '.') {
            if (!digits || parts > 3) return EFalse;
            aIp = (aIp << 8) | (TUint32)part;
            parts++;
            part = 0;
            digits = 0;
        } else if (aIn[i] >= '0' && aIn[i] <= '9') {
            part = part * 10 + (aIn[i] - '0');
            digits++;
            if (part > 255 || digits > 3) return EFalse;
        } else {
            return EFalse;
        }
    }
    return parts == 4;
}

class CRunner;

// Active object doing one UDP receive at a time.
class CRecv : public CActive
{
public:
    CRecv(CRunner& aOwner, TInt aKind) : CActive(CActive::EPriorityStandard), iOwner(aOwner),
        iKind(aKind), iPtr(iBuf, 0, KMaxPkt)
    { CActiveScheduler::Add(this); }
    ~CRecv() { Cancel(); }
    void Start(RSocket& aSock)
    {
        iSock = &aSock;
        iPtr.Zero();
        aSock.RecvFrom(iPtr, iFrom, 0, iStatus);
        SetActive();
    }
    void RunL();
    void DoCancel() { if (iSock) iSock->CancelRecv(); }
private:
    CRunner& iOwner;
    TInt iKind;
    RSocket* iSock;
    TUint8 iBuf[KMaxPkt];
    TPtr8 iPtr;
    TInetAddr iFrom;
};

// Receiver for the encoder units from tgvideo.exe (big datagrams).
class CVRecv : public CActive
{
public:
    CVRecv(CRunner& aOwner) : CActive(CActive::EPriorityStandard), iOwner(aOwner), iPtr(iBuf, 0, KVideoMax)
    { CActiveScheduler::Add(this); }
    ~CVRecv() { Cancel(); }
    void Start(RSocket& aSock)
    {
        iSock = &aSock;
        iPtr.Zero();
        aSock.RecvFrom(iPtr, iFrom, 0, iStatus);
        SetActive();
    }
    void RunL();
    void DoCancel() { if (iSock) iSock->CancelRecv(); }
private:
    CRunner& iOwner;
    RSocket* iSock;
    TUint8 iBuf[KVideoMax];
    TPtr8 iPtr;
    TInetAddr iFrom;
};

// Active object sending one UDP datagram at a time (drops if busy).
class CSend : public CActive
{
public:
    CSend() : CActive(CActive::EPriorityStandard), iPtr(iBuf, 0, KMaxPkt)
    { CActiveScheduler::Add(this); }
    ~CSend() { Cancel(); }
    TBool Send(RSocket& aSock, const TInetAddr& aTo, const TUint8* aData, TInt aLen)
    {
        if (IsActive()) return EFalse;
        iSock = &aSock;
        iTo = aTo;
        Mem::Copy(iBuf, aData, aLen);
        iPtr.Set(iBuf, aLen, KMaxPkt);
        aSock.SendTo(iPtr, iTo, 0, iStatus);
        SetActive();
        return ETrue;
    }
    void RunL() { iResult = iStatus.Int(); }
    void DoCancel() { if (iSock) iSock->CancelSend(); }
    TInt iResult;
private:
    RSocket* iSock;
    TInetAddr iTo;
    TUint8 iBuf[KMaxPkt];
    TPtr8 iPtr;
};

class CRunner : public CBase, public MVoIPDownlinkObserver,
                public MVoIPUplinkObserver, public MVoIPFormatObserver
{
public:
    CRunner(TInt aSeconds, TBool aRelay) : iSeconds(aSeconds), iRelayMode(aRelay), iWantRoute(1) {}
    ~CRunner()
    {
        delete iTimer;
        delete iTimer20;
        delete iRelayRecv;
        iRelaySock.Close();
        iConn.Close();
        for (TInt i = 0; i < KSends; i++) delete iSends[i];
        delete [] (TUint8*)iCtx;
        delete [] (TUint8*)iS;
        delete [] iDnPcm;
        if (iUp) { iUp->Stop(); iUp->Close(); }
        if (iDn) { iDn->Stop(); iDn->Close(); }
        delete iUpFmt;
        delete iDnFmt;
        delete iUp;
        delete iDn;
        delete iPlayBuf;
        delete iFactory;
        iFormats.Close();
        delete iRecv;
        delete iVRecv;
        iVSock.Close();
        iSock.Close();
        iSs.Close();
        if (iEnc) opus_encoder_destroy(iEnc);
        if (iDec) opus_decoder_destroy(iDec);
        delete [] iRing;
        Write(_L8("tgcallaudio end\n"));
        iFile.Close();
        iFs.Close();
    }
    void StartL()
    {
        User::LeaveIfError(iFs.Connect());
        _LIT(KCard, "E:\\retrotele_callaudio.txt");
        _LIT(KPhone, "C:\\Data\\retrotele_callaudio.txt");
        TInt e = iFile.Open(iFs, KCard, EFileWrite | EFileShareAny);
        if (e == KErrNone) {
            TInt sz = 0;
            iFile.Size(sz);
            iFile.Seek(ESeekEnd, sz);
        } else if (iFile.Replace(iFs, KCard, EFileWrite | EFileShareAny) != KErrNone) {
            e = iFile.Open(iFs, KPhone, EFileWrite | EFileShareAny);
            if (e == KErrNone) {
                TInt sz = 0;
                iFile.Size(sz);
                iFile.Seek(ESeekEnd, sz);
            } else {
                User::LeaveIfError(iFile.Replace(iFs, KPhone, EFileWrite | EFileShareAny));
            }
        }
        Write(_L8("tgcallaudio start\n"));
        iRing = new TUint8[KRingSize * KMaxPkt];
        if (!iRing) User::Leave(KErrNoMemory);
        Mem::FillZ(iRingLen, sizeof(iRingLen));
        iTimer = CPeriodic::NewL(CActive::EPriorityStandard);
        iTimer->Start(1000000, 1000000, TCallBack(Tick, this));
        SetupOpus();
        Write(_L8("step: opus ready\n"));
        SetupNet();
        Write(_L8("step: local socket ready\n"));
        if (iRelayMode) {
            Write(_L8("step: relay setup\n"));
            SetupRelay();
            iTimer20 = CPeriodic::NewL(CActive::EPriorityStandard);
            iTimer20->Start(20000, 20000, TCallBack(Tick20, this));
        }
        Write(_L8("step: starting audio\n"));
        SetupAudio();
        Write(_L8("step: audio requested\n"));
    }
    TBool Check(const TDesC8& aStep, TInt aErr)
    {
        if (aErr != KErrNone) {
            iErrors++;
            iLast = aErr;
            if (iFirstErrs < 8) {
                iFirstErrs++;
                TBuf8<96> b;
                b.Format(_L8("error %S %d\n"), &aStep, aErr);
                Write(b);
            }
            return EFalse;
        }
        return ETrue;
    }
    void SetupOpus()
    {
        int err = 0;
        iEnc = opus_encoder_create(8000, 1, OPUS_APPLICATION_VOIP, &err);
        if (!iEnc || err != OPUS_OK) { Check(_L8("EncCreate"), err ? err : KErrNoMemory); iEnc = NULL; return; }
        opus_encoder_ctl(iEnc, OPUS_SET_BITRATE(12000));
        opus_encoder_ctl(iEnc, OPUS_SET_COMPLEXITY(2));
        opus_encoder_ctl(iEnc, OPUS_SET_VBR(0));
        opus_encoder_ctl(iEnc, OPUS_SET_INBAND_FEC(0));
        opus_encoder_ctl(iEnc, OPUS_SET_SIGNAL(OPUS_SIGNAL_VOICE));
        opus_encoder_ctl(iEnc, OPUS_SET_PACKET_LOSS_PERC(0));
        iDec = opus_decoder_create(8000, 1, &err);
        if (!iDec || err != OPUS_OK) { Check(_L8("DecCreate"), err ? err : KErrNoMemory); iDec = NULL; }
    }
    void SetupNet()
    {
        if (!Check(_L8("SocketServ"), iSs.Connect())) return;
        if (!Check(_L8("SocketOpen"), iSock.Open(iSs, KAfInet, KSockDatagram, KProtocolInetUdp))) return;
        TInetAddr local;
        local.SetAddress(INET_ADDR(127, 0, 0, 1));
        local.SetPort(KPortIn);
        if (!Check(_L8("SocketBind"), iSock.Bind(local))) return;
        iSock.SetOpt(KSORecvBuf, KSOLSocket, 32768);   /* queue bursts instead of dropping */
        iSock.SetOpt(KSOSendBuf, KSOLSocket, 16384);
        iDest.SetAddress(INET_ADDR(127, 0, 0, 1));
        iDest.SetPort(KPortOut);
        for (TInt i = 0; i < KSends; i++) {
            iSends[i] = new CSend;
            if (!iSends[i]) { Check(_L8("NetAlloc"), KErrNoMemory); return; }
        }
        iRecv = new CRecv(*this, 0);
        if (!iRecv) { Check(_L8("NetAlloc"), KErrNoMemory); return; }
        iRecv->Start(iSock);
        iNetOk = ETrue;
        iTgv.SetAddress(INET_ADDR(127, 0, 0, 1));
        iTgv.SetPort(KPortVideoCmd);
        iTgvRx.SetAddress(INET_ADDR(127, 0, 0, 1));
        iTgvRx.SetPort(KPortVideoRx);
        // encoder units from tgvideo.exe; failure here only disables video, never the call
        if (Check(_L8("VSockOpen"), iVSock.Open(iSs, KAfInet, KSockDatagram, KProtocolInetUdp))) {
            TInetAddr vl;
            vl.SetAddress(INET_ADDR(127, 0, 0, 1));
            vl.SetPort(KPortVideoIn);
            if (Check(_L8("VSockBind"), iVSock.Bind(vl))) {
                iVSock.SetOpt(KSORecvBuf, KSOLSocket, 65536);
                iVRecv = new CVRecv(*this);
                if (iVRecv) { iVRecv->Start(iVSock); iVNet = ETrue; }
            }
        }
    }
    void SetupAudio()
    {
        if (!Check(_L8("CreateFactory"), CVoIPUtilityFactory::CreateFactory(iFactory))) return;
        if (!Check(_L8("CreateDownlink"), iFactory->CreateDownlinkStream(TVersion(1, 0, 0),
                   CVoIPUtilityFactory::EVoIPCall, iDn))) return;
        if (!Check(_L8("CreateUplink"), iFactory->CreateUplinkStream(TVersion(1, 0, 0),
                   CVoIPUtilityFactory::EVoIPCall, iUp))) return;
        if (!Check(_L8("CreateBuffer"), iFactory->CreateBuffer(iPlayBuf,
                   CVoIPDataBuffer::EStandard))) return;
        if (!Check(_L8("DnSetFormat"), iDn->SetFormat(EPCM16, iDnFmt))) return;
        Check(_L8("DnFmtObserver"), iDnFmt->SetObserver(*this));
        if (!Check(_L8("UpSetFormat"), iUp->SetFormat(EPCM16, iUpFmt))) return;
        Check(_L8("UpFmtObserver"), iUpFmt->SetObserver(*this));
        if (!Check(_L8("DnOpen"), iDn->Open(*this))) return;
        Check(_L8("UpOpen"), iUp->Open(*this));
    }
    void MaybeStart()
    {
        if (!iDnOpen || !iUpOpen || iStarted) return;
        iStarted = ETrue;
        TInt mv = 0, mg = 0;
        Check(_L8("SetAudioDevice"), iDn->SetAudioDevice(RouteDev(iWantRoute)));
        iDn->GetMaxVolume(mv);
        Check(_L8("SetVolume"), iDn->SetVolume(mv * 70 / 100));
        iUp->GetMaxGain(mg);
        Check(_L8("SetGain"), iUp->SetGain(mg));
        if (!Check(_L8("DnStart"), iDn->Start())) return;
        Check(_L8("UpStart"), iUp->Start());
    }
    // ---- network receive; returns ETrue to re-arm the receive at once
    TBool Received(TInt aKind, TInt aErr, const TDesC8& aData, const TInetAddr& aFrom)
    {
        if (aKind == 1) {
            if (aErr == KErrNone) {
                HandleRelay(aData, aFrom);
                return ETrue;
            }
            if (aErr == KErrCancel || aErr == KErrAbort) return EFalse;
            Check(_L8("RelayRecv"), aErr);   /* e.g. ICMP unreachable: retry on the 20 ms tick */
            return EFalse;
        }
        if (aErr == KErrNone) {
            TInt n = aData.Length();
            if (n == 1 && aData[0] == 'Q') {
                CActiveScheduler::Stop();
                return EFalse;
            }
            if (n > 1 && aData[0] == 'G') {
                if (iV277 && iS && iRelayOk) {
                    TInt e = c277_session_on_signaling(iS, aData.Ptr() + 1, (u32)(n - 1), NowMs());
                    iSigIn++;
                    if (e != 0) { Check(_L8("Signaling"), e); }
                    Drain277();
                }
                return ETrue;
            }
            if (n == 2 && aData[0] == 'V' && (aData[1] == '0' || aData[1] == '1')) {
                TBuf8<48> lb;
                lb.Format(_L8("control: video %d\n"), aData[1] == '1' ? 1 : 0);
                Write(lb);
                if (iV277 && iS) {
                    c277_session_video_enable(iS, aData[1] == '1');
                    if (aData[1] == '1') iVideoEver = ETrue;
                }
                return ETrue;
            }
            if (n == 2 && (aData[0] == 'W' || aData[0] == 'H')) {
                if (aData[0] == 'W') iRxWant = (aData[1] == '1'); else iRxFwd = (aData[1] == '1');
                TBuf8<48> lb;
                lb.Format(_L8("control: peer video %c %d\n"), (TInt)aData[0], aData[1] == '1' ? 1 : 0);
                Write(lb);
                ApplyRx();
                return ETrue;
            }
            if (n == 2 && aData[0] == 'M') {
                iMuted = (aData[1] == '1');
                TBuf8<48> lb;
                lb.Format(_L8("control: mute %d\n"), iMuted ? 1 : 0);
                Write(lb);
                SendInfo('M', iMuted ? 1 : 0);
                return ETrue;
            }
            if (n == 2 && aData[0] == 'R' && aData[1] >= '0' && aData[1] <= '2') {
                iWantRoute = aData[1] - '0';
                TInt rerr = KErrNone;
                if (iStarted && iDn) rerr = iDn->SetAudioDevice(RouteDev(iWantRoute));
                TBuf8<64> lb;
                lb.Format(_L8("control: route %d err %d\n"), iWantRoute, rerr);
                Write(lb);
                if (iStarted) ReportRoute();
                return ETrue;
            }
            if (!iRelayMode && n > 0 && n <= KMaxPkt) {
                iRecvPkts++;
                if (iRingCount == KRingSize) { // drop oldest
                    iRingHead = (iRingHead + 1) % KRingSize;
                    iRingCount--;
                }
                TInt slot = (iRingHead + iRingCount) % KRingSize;
                Mem::Copy(iRing + slot * KMaxPkt, aData.Ptr(), n);
                iRingLen[slot] = n;
                iRingCount++;
                iEverRecv = ETrue;
            }
        } else {
            Check(_L8("RecvFrom"), aErr);
            if (aErr == KErrCancel || aErr == KErrAbort) return EFalse;
        }
        return ETrue;
    }
    // ---- relay mode
    TBool SendTo(RSocket& aSock, const TInetAddr& aTo, const TUint8* aData, TInt aLen)
    {
        for (TInt i = 0; i < KSends; i++) {
            if (iSends[i] && !iSends[i]->IsActive()) {
                if (iSends[i]->iResult != KErrNone && iSends[i]->iResult != 0) Check(_L8("SendTo"), iSends[i]->iResult);
                iSends[i]->iResult = 0;
                return iSends[i]->Send(aSock, aTo, aData, aLen);
            }
        }
        Check(_L8("SendBusy"), KErrInUse);
        return EFalse;
    }
    void SendStatus(const TDesC8& aName)
    {
        if (!iNetOk) return;
        TBuf8<24> b;
        b.Append(_L8("S "));
        b.Append(aName);
        SendTo(iSock, iDest, b.Ptr(), b.Length());
    }
    void SendInfo(TUint8 aTag, TInt aVal)
    {
        if (!iNetOk) return;
        TBuf8<8> b;
        b.Append(aTag);
        b.Append(' ');
        b.AppendNum(aVal);
        SendTo(iSock, iDest, b.Ptr(), b.Length());
    }
    static CVoIPAudioDownlinkStream::TVoIPOutputDevice RouteDev(TInt aRoute)
    {
        if (aRoute == 0) return CVoIPAudioDownlinkStream::EHandset;
        if (aRoute == 1) return CVoIPAudioDownlinkStream::ELoudSpeaker;
        return CVoIPAudioDownlinkStream::ENoPreference;
    }
    void ReportRoute()
    {
        if (!iDn) return;
        CVoIPAudioDownlinkStream::TVoIPOutputDevice d = CVoIPAudioDownlinkStream::ENoPreference;
        TInt e = iDn->GetAudioDevice(d);
        TInt v = 2;
        if (e == KErrNone) {
            if (d == CVoIPAudioDownlinkStream::EHandset) v = 0;
            else if (d == CVoIPAudioDownlinkStream::ELoudSpeaker) v = 1;
        }
        TBuf8<64> lb;
        lb.Format(_L8("audio route now %d (err %d)\n"), v, e);
        Write(lb);
        SendInfo('D', v);
    }
    void FinalStatus()
    {
        if (iVideoEver && iNetOk) {      // never leave the camera reserved: ask tgvideo.exe to quit
            TRequestStatus qs;
            TBuf8<2> q(_L8("Q"));
            iSock.SendTo(q, iTgv, 0, qs);
            User::WaitForRequest(qs);
        }
        if (!iRelayMode || !iNetOk) return;
        TBuf8<24> b(_L8("S ended"));
        TRequestStatus st;
        iSock.SendTo(b, iDest, 0, st);
        User::WaitForRequest(st);
    }
    void Event8(const TDesC8& aText) { Write(aText); }
    void SetupRelay()
    {
        iLocked = -1;
        Write(_L8("relay: 1 clear endpoints\n"));
        for (TInt z = 0; z < KMaxEp; z++) Mem::FillZ(iEp[z].tag, 16);   /* tags only: TInetAddr must keep its own state */
        Write(_L8("relay: 2 allocate protocol context\n"));
        iCtx = (tgcp_ctx*)new TUint8[sizeof(tgcp_ctx)];
        if (!iCtx) { Check(_L8("CtxAlloc"), KErrNoMemory); return; }
        Mem::FillZ(iCtx, sizeof(tgcp_ctx));
        Write(_L8("relay: 3 read config\n"));
        if (!ReadConfig()) {
            Write(_L8("no usable call config\n"));
            return;
        }
        {
            TBuf8<64> cb;
            cb.Format(_L8("step: config ok, %d endpoints, iap %d\n"), iNEp, iIap);
            Write(cb);
        }
        // open our own connection on the access point RetroTele uses: no
        // dialog, and it is shared when that connection is already up
        TInt cerr = iConn.Open(iSs);
        if (cerr == KErrNone) {
            if (iIap > 0) {
                TCommDbConnPref pref;
                pref.SetIapId(iIap);
                pref.SetDialogPreference(ECommDbDialogPrefDoNotPrompt);
                pref.SetDirection(ECommDbConnectionDirectionOutgoing);
                cerr = iConn.Start(pref);
            } else {
                cerr = iConn.Start();
            }
            if (cerr == KErrNone) iConnOk = ETrue; else iConn.Close();
        }
        {
            TBuf8<64> cb;
            cb.Format(_L8("step: connection %S (%d)\n"), iConnOk ? &_L8("up") : &_L8("none"), cerr);
            Write(cb);
        }
        TInt e = iConnOk ? iRelaySock.Open(iSs, KAfInet, KSockDatagram, KProtocolInetUdp, iConn)
                         : iRelaySock.Open(iSs, KAfInet, KSockDatagram, KProtocolInetUdp);
        if (!Check(_L8("RelayOpen"), e)) return;
        TInetAddr any;
        any.SetPort(0);
        Check(_L8("RelayBind"), iRelaySock.Bind(any));
        iRelaySock.SetOpt(KSORecvBuf, KSOLSocket, 16384);
        iRelaySock.SetOpt(KSOSendBuf, KSOLSocket, 16384);
        iRelayRecv = new CRecv(*this, 1);
        if (!iRelayRecv) { Check(_L8("RelayAlloc"), KErrNoMemory); return; }
        if (iV277) {
            Setup277();
            return;
        }
        tgcp_init(iCtx, iKey, iOut, iEp[0].tag);
        tgcp_set_now(iCtx, NowMs());
        iRelayRecv->Start(iRelaySock);
        iRelayOk = ETrue;
        iStatusName.Copy(_L8("connecting"));
        SendStatus(iStatusName);
        TBuf8<96> b;
        b.Format(_L8("relay start out=%d endpoints=%d\n"), iOut, iNEp);
        Write(b);
    }
    // ---- v277 (official apps) session
    void Setup277()
    {
        iS = (c277_session*)new TUint8[sizeof(c277_session)];
        iDnPcm = new TInt16[KDnRing];
        if (!iS || !iDnPcm) { Check(_L8("SessAlloc"), KErrNoMemory); return; }
        Mem::FillZ(iS, sizeof(c277_session));
        TInt r = c277_session_init(iS, iKey, iOut, iConn277, (u32)iNConn, iRnd, 56);
        Mem::FillZ(iKey, sizeof(iKey));
        Mem::FillZ(iRnd, sizeof(iRnd));
        {
            TBuf8<96> b;
            b.Format(_L8("step: v277 session init %d conns %d\n"), r, iNConn);
            Write(b);
        }
        if (r == C277_SESS_OK) SetWallClock();
        if (r == C277_SESS_OK) ApplyRx();
        if (r == C277_SESS_OK) {
            _LIT(KFlag, "E:\\retrotele_videocap.flag");
            TEntry ent;
            if (iFs.Entry(KFlag, ent) == KErrNone) {
                c277_session_capture(iS, 1, 0);      /* the flag only turns the log on: it never makes us announce video */
                iCap = ETrue;
                CapWrite(_L8("--- v277 call, video capture on (no key, no peer IPs logged)\n"));
                Write(_L8("step: video capture flag present\n"));
            }
        }
        if (r != C277_SESS_OK) { Check(_L8("SessInit"), r); iStatusName.Copy(_L8("failed")); SendStatus(iStatusName); return; }
        iRelayRecv->Start(iRelaySock);
        iRelayOk = ETrue;
        iStatusName.Copy(_L8("connecting"));
        SendStatus(iStatusName);
        SendStatus(_L8("ready"));
        Write(_L8("relay start v277\n"));
    }
    void CapWrite(const TDesC8& aText)
    {
        _LIT(KVid, "E:\\retrotele_callvideo.txt");
        RFile f;
        TInt e = f.Open(iFs, KVid, EFileWrite | EFileShareAny);
        if (e == KErrNotFound) e = f.Replace(iFs, KVid, EFileWrite | EFileShareAny);
        if (e != KErrNone) return;
        TInt sz = 0;
        f.Size(sz);
        f.Seek(ESeekEnd, sz);
        f.Write(aText);
        f.Close();
    }
    void FlushCap()
    {
        if (!iCap || !iS || iS->cap_len == 0) return;
        char buf[1024];
        TInt tot = 0;
        while (iS->cap_len > 0 && tot < 8192) {
            u32 n = c277_session_cap_take(iS, buf, sizeof(buf));
            CapWrite(TPtrC8((const TUint8*)buf, (TInt)n));
            tot += (TInt)n;
        }
    }
    void Drain277()
    {
        FlushCap();
        for (TInt i = 0; i < iS->out.n; i++) {
            c277_out_item& it = iS->out.item[i];
            if (it.kind == C277_OUT_UDP) {
                TInetAddr to(it.ip, (TUint)it.port);
                SendTo(iRelaySock, to, it.data, (TInt)it.len);
                iUdpTx++;
            } else if (it.kind == C277_OUT_SIGNAL && iNetOk) {
                TUint8 tmp[C277_OUT_MAX + 1];
                tmp[0] = 'G';
                Mem::Copy(tmp + 1, it.data, it.len);
                SendTo(iSock, iDest, tmp, (TInt)it.len + 1);
                iSigOut++;
            }
        }
        iS->out.n = 0;
    }
    void PollOpus277()
    {
        if (!iDec || !iS) return;
        for (TInt k = 0; k < 8; k++) {
            if (KDnRing - iDnCount < 960) break;       /* ring full: leave packets queued */
            TUint8 pkt[KOpusMax];
            u32 len = 0;
            TInt r = c277_session_poll_opus(iS, pkt, sizeof(pkt), &len);
            if (r == C277_JB_EMPTY) break;
            TInt16 pcm[960];
            TInt n;
            if (r == C277_JB_PACKET) {
                n = opus_decode(iDec, pkt, (opus_int32)len, pcm, 960, 0);
                iRecvPkts++;
            } else {
                n = opus_decode(iDec, NULL, 0, pcm, 480, 0);
                iLostPkts++;
                iPlcFrames++;
            }
            if (n < 0) { Check(_L8("Decode"), n); continue; }
            for (TInt i = 0; i < n; i++) {
                iDnPcm[(iDnHead + iDnCount) % KDnRing] = pcm[i];
                iDnCount++;
            }
            iEverRecv = ETrue;
        }
    }
    // ---- video send
    void VideoUnit(const TDesC8& aData)
    {
        iVUnits++;
        if (iVUnits == 1 || iVUnits == 50) {
            TBuf8<64> lb;
            lb.Format(_L8("video: unit %d from tgvideo.exe, %d bytes\n"), iVUnits, aData.Length());
            Write(lb);
        }
        if (!iV277 || !iS || !iRelayOk || aData.Length() < 2 || aData[0] != 'V') return;
        c277_session_video_put(iS, aData.Ptr() + 1, (u32)(aData.Length() - 1), NowMs());
        PumpVideo();
    }
    void PumpVideo()
    {
        for (TInt i = 0; i < 12; i++) {
            TInt more = c277_session_video_pump(iS, NowMs());
            Drain277();
            if (!more) break;
        }
        CheckIdrRequest();
    }
    void ApplyRx()
    {
        if (iV277 && iS) c277_session_rx_enable(iS, iRxWant, iRxFwd, 150);
    }
    // peer video -> tgvideo.exe (port 47015). 'I' text line "I profile level w h" when the peer's SPS
    // changes, then each access unit as fragments: 'R', id, index, count, idr, up to 1200 bytes.
    void PumpRx()
    {
        c277_sps sp;
        const u8* au = NULL;
        u32 len = 0;
        TInt idr = 0;
        if (!iS || !iNetOk || !iV277) return;
        if (c277_session_vrx_sps_take(iS, &sp)) {
            TBuf8<64> b;
            b.Format(_L8("I %d %d %d %d %d"), sp.profile, sp.constraints, sp.level, sp.width, sp.height);
            SendTo(iSock, iTgvRx, b.Ptr(), b.Length());
            TBuf8<96> lb;
            lb.Format(_L8("rx: peer SPS profile %d level %d size %dx%d sent to tgvideo\n"), sp.profile, sp.level, sp.width, sp.height);
            Write(lb);
        }
        for (TInt guard = 0; guard < 6 && c277_session_vrx_take(iS, &au, &len, &idr); guard++) {
            TUint8 pk[1210];
            TInt cnt = (TInt)((len + 1199) / 1200);
            if (cnt > 40) cnt = 40;
            iRxId = (iRxId + 1) & 0xFF;
            for (TInt i = 0; i < cnt; i++) {
                TInt off = i * 1200, n = (TInt)len - off;
                if (n > 1200) n = 1200;
                pk[0] = 'R'; pk[1] = (TUint8)iRxId; pk[2] = (TUint8)i; pk[3] = (TUint8)cnt; pk[4] = (TUint8)(idr ? 1 : 0);
                Mem::Copy(pk + 5, au + off, n);
                SendTo(iSock, iTgvRx, pk, n + 5);
            }
            iRxAu++;
            if (iRxAu <= 5 || iRxAu % 100 == 0) {
                TBuf8<96> fl;
                fl.Format(_L8("rx: forwarded unit %d (%d bytes, idr %d, %d fragments) to tgvideo\n"), iRxAu, (TInt)len, idr, cnt);
                Write(fl);
            }
            c277_session_vrx_done(iS);
        }
    }
    void CheckIdrRequest()
    {
        if (!iS || !iNetOk) return;
        if (c277_session_take_idr_req(iS)) {
            SendTo(iSock, iTgv, (const TUint8*)"K", 1);
            iKeyReqs++;
        }
    }
    void VideoInfo()
    {
        if (!iS || !iNetOk) return;
        if (iS->peer_vid != iLastPeerVid) {
            iLastPeerVid = iS->peer_vid;
            if (iLastPeerVid >= 0) SendInfo('P', iLastPeerVid);
        }
        if ((TInt)iS->req_video_n != iLastReqVid) {
            iLastReqVid = (TInt)iS->req_video_n;
            SendInfo('X', iLastReqVid);
        }
        TInt act = c277_session_video_active(iS) ? 1 : 0;
        if (act != iLastVidAct) {
            iLastVidAct = act;
            SendInfo('A', act);
            TBuf8<48> lb;
            lb.Format(_L8("event: video send %d\n"), act);
            Write(lb);
        }
    }
    void SetWallClock()
    {
        TTime now;
        now.UniversalTime();
        TTime epoch(TDateTime(1970, EJanuary, 0, 0, 0, 0, 0));
        TTimeIntervalSeconds sec;
        if (now.SecondsFrom(epoch, sec) == KErrNone) c277_session_set_wall(iS, (u32)sec.Int(), NowMs());
    }
    void Handle277(const TDesC8& aData, const TInetAddr& aFrom)
    {
        if (!iS) return;
        c277_session_on_udp(iS, aFrom.Address(), aFrom.Port(), aData.Ptr(), (u32)aData.Length(), NowMs());
        PollOpus277();
        Drain277();
        CheckIdrRequest();      /* a PLI/FIR from the peer */
        PumpRx();
    }
    void Tick277()
    {
        if (!iS) return;
        c277_session_tick(iS, NowMs());
        PollOpus277();
        Drain277();
        CheckIdrRequest();
        VideoInfo();
        PumpRx();
        TInt ice = c277_session_ice_state(iS);
        if (ice != iLastIce) {
            TBuf8<48> b;
            b.Format(_L8("event: ice %d -> %d\n"), iLastIce, ice);
            Write(b);
            iLastIce = ice;
        }
        if (!iEvCreds && iS->ice.have_creds) { iEvCreds = ETrue; Write(_L8("event: peer credentials\n")); }
        if (!iEvCand && iS->peer_cands > 0) { iEvCand = ETrue; Write(_L8("event: peer candidate\n")); }
        if (!iEvRefl && c277_session_refl_ready(iS)) { iEvRefl = ETrue; Write(_L8("event: reflector hello reply\n")); }
        if (!iEvRtp && iS->n_rtp_rx > 0) { iEvRtp = ETrue; Write(_L8("event: first RTP packet\n")); }
        TBool conn = c277_session_ice_connected(iS) && c277_session_media_flowing(iS, NowMs());
        if (conn) iEverConn = ETrue;
        TPtrC8 want(_L8("connecting"));
        if (conn) want.Set(_L8("connected"));
        else if (iEverConn) want.Set(_L8("reconnecting"));
        else if (ice == C277_ICE_FAILED) want.Set(_L8("failed"));
        if (iStatusName != want) {
            TBuf8<64> b;
            b.Format(_L8("state %S -> %S\n"), &iStatusName, &want);
            Write(b);
            iStatusName.Copy(want);
            SendStatus(iStatusName);
        }
    }
    TBool Media277() { return iV277 && iS && iRelayOk && c277_session_ice_connected(iS); }
    TBool ReadConfig()
    {
        _LIT(KC, "C:\\Data\\retrotele_call.cfg");
        _LIT(KE, "E:\\retrotele_call.cfg");
        TBuf8<2048> buf;
        TBool got = EFalse;
        RFile f;
        TInt oe = f.Open(iFs, KC, EFileRead | EFileShareAny);
        {
            TBuf8<64> lb;
            lb.Format(_L8("cfg: open C: %d\n"), oe);
            Write(lb);
        }
        if (oe == KErrNone) {
            TInt re = f.Read(buf);
            got = re == KErrNone;
            f.Close();
            iFs.Delete(KC);
            TBuf8<64> lb;
            lb.Format(_L8("cfg: read C: %d len %d\n"), re, buf.Length());
            Write(lb);
        }
        if (!got) {
            oe = f.Open(iFs, KE, EFileRead | EFileShareAny);
            TBuf8<64> lb;
            lb.Format(_L8("cfg: open E: %d\n"), oe);
            Write(lb);
            if (oe == KErrNone) {
                got = f.Read(buf) == KErrNone;
                f.Close();
                iFs.Delete(KE);
            }
        }
        if (!got) return EFalse;
        Write(_L8("cfg: parsing\n"));
        iNEp = 0;
        TBool haveKey = EFalse, haveRnd = EFalse;
        iNConn = 0;
        const TUint8* p = buf.Ptr();
        TInt n = buf.Length();
        TInt pos = 0;
        while (pos < n) {
            TInt s = pos;
            while (pos < n && p[pos] != '\n') pos++;
            TInt e = pos;
            pos++;
            while (e > s && (p[e - 1] == '\r' || p[e - 1] == ' ')) e--;
            const TUint8* l = p + s;
            TInt len = e - s;
            if (len == 7 && l[0] == 'v' && l[1] == 'e' && l[2] == 'r' && l[3] == '=' &&
                l[4] == '2' && l[5] == '7' && l[6] == '7') {
                iV277 = ETrue;
            } else if (len == 4 + 112 && l[0] == 'r' && l[1] == 'n' && l[2] == 'd' && l[3] == '=') {
                haveRnd = ParseHex(l + 4, 112, iRnd);
            } else if (len > 2 && l[0] == 'c' && l[1] == '=' && iNConn < KMaxConn) {
                ParseConn(l + 2, len - 2);
            } else if (len > 4 && l[0] == 'i' && l[1] == 'a' && l[2] == 'p' && l[3] == '=') {
                TInt v = 0;
                if (ParseInt(l + 4, len - 4, v)) iIap = v;
            } else if (len > 4 && l[0] == 'o' && l[1] == 'u' && l[2] == 't' && l[3] == '=') {
                iOut = l[4] == '1' ? 1 : 0;
            } else if (len == 4 + 512 && l[0] == 'k' && l[1] == 'e' && l[2] == 'y' && l[3] == '=') {
                haveKey = ParseHex(l + 4, 512, iKey);
            } else if (len > 3 && l[0] == 'e' && l[1] == 'p' && l[2] == '=' && iNEp < KMaxEp) {
                // ip,port,tag32hex
                TInt c1 = 3;
                while (c1 < len && l[c1] != ',') c1++;
                TInt c2 = c1 + 1;
                while (c2 < len && l[c2] != ',') c2++;
                TUint32 ip;
                TInt port;
                if (c2 < len && ParseIp(l + 3, c1 - 3, ip) && ParseInt(l + c1 + 1, c2 - c1 - 1, port) &&
                    port > 0 && port < 65536 && len - c2 - 1 == 32 &&
                    ParseHex(l + c2 + 1, 32, iEp[iNEp].tag)) {
                    iEp[iNEp].addr = TInetAddr(ip, (TUint)port);
                    iNEp++;
                }
            }
        }
        Mem::FillZ((TUint8*)buf.Ptr(), 2048);   /* do not leave the key around */
        {
            TBuf8<64> lb;
            lb.Format(_L8("cfg: parsed key=%d eps=%d conns=%d rnd=%d v277=%d\n"), haveKey, iNEp, iNConn,
                      haveRnd, iV277);
            Write(lb);
        }
        if (iV277) return haveKey && haveRnd && iNConn > 0;
        return haveKey && iNEp > 0;
    }
    // c=<id64 decimal>,<ip>,<port>,<tag32hex or empty>,<tcp>,<webrtc>
    void ParseConn(const TUint8* l, TInt len)
    {
        TInt cp[5], nc = 0;
        for (TInt i = 0; i < len && nc < 5; i++) if (l[i] == ',') cp[nc++] = i;
        if (nc != 5) return;
        c277_sess_conn& c = iConn277[iNConn];
        Mem::FillZ(&c, sizeof(c));
        // signed 64-bit decimal -> hi (signed) / lo, via 32-bit limbs
        TInt i = 0;
        TBool neg = EFalse;
        if (i < cp[0] && l[i] == '-') { neg = ETrue; i++; }
        if (i >= cp[0] || cp[0] - i > 20) return;
        TUint32 hi = 0, lo = 0;
        for (; i < cp[0]; i++) {
            if (l[i] < '0' || l[i] > '9') return;
            TUint32 d = (TUint32)(l[i] - '0');
            // (hi:lo) = (hi:lo) * 10 + d in 16-bit steps
            TUint32 a = (lo & 0xFFFF) * 10 + d;
            TUint32 b = (lo >> 16) * 10 + (a >> 16);
            lo = (b << 16) | (a & 0xFFFF);
            hi = hi * 10 + (b >> 16);
        }
        if (neg) {
            lo = ~lo + 1;
            hi = ~hi + (lo == 0 ? 1 : 0);
        }
        c.id.hi = (TInt)hi;
        c.id.lo = lo;
        TUint32 ip;
        TInt port;
        if (!ParseIp(l + cp[0] + 1, cp[1] - cp[0] - 1, ip)) return;
        if (!ParseInt(l + cp[1] + 1, cp[2] - cp[1] - 1, port) || port < 0 || port > 65535) return;
        TInt tl = cp[3] - cp[2] - 1;
        if (tl == 32) {
            if (!ParseHex(l + cp[2] + 1, 32, c.peer_tag)) return;
        } else if (tl != 0) {
            return;
        }
        c.ip = ip;
        c.port = (TUint32)port;
        if (cp[4] + 1 >= len) return;
        c.tcp = l[cp[3] + 1] == '1';
        c.webrtc = l[cp[4] + 1] == '1';
        iNConn++;
    }
    TInt FindEp(const TInetAddr& aFrom)
    {
        for (TInt i = 0; i < iNEp; i++)
            if (iEp[i].addr.Address() == aFrom.Address() && iEp[i].addr.Port() == aFrom.Port()) return i;
        return -1;
    }
    void LogEp(const TDesC8& aWhat, TInt aIdx)
    {
        TUint32 a = iEp[aIdx].addr.Address();
        TBuf8<128> b;
        b.Format(_L8("%S ep%d %d.%d.%d.%d:%d\n"), &aWhat, aIdx, (a >> 24) & 255, (a >> 16) & 255,
                 (a >> 8) & 255, a & 255, (TInt)iEp[aIdx].addr.Port());
        Write(b);
    }
    void HandleRelay(const TDesC8& aData, const TInetAddr& aFrom)
    {
        if (iV277) { if (iRelayOk) Handle277(aData, aFrom); return; }
        if (!iRelayOk || !iCtx) return;
        TInt idx = FindEp(aFrom);
        if (idx < 0) return;
        const TUint8* d = aData.Ptr();
        TInt n = aData.Length();
        u8 qid[8];
        TInt refl = tgcp_is_reflector_reply(d, (u32)n, qid);
        if (refl) {
            if (refl == 1) {
                iReflectorRx++;
                if (qid[0] == (TUint8)idx) {
                    iRtt[idx] = (TInt)(NowMs() - iPingMs[idx]);
                    if (!iPongSeen[idx]) {
                        iPongSeen[idx] = ETrue;
                        LogEp(_L8("reflector reply"), idx);
                    }
                }
            }
            return;
        }
        if (iLocked >= 0 && idx != iLocked) return;
        if (iLocked < 0) tgcp_set_peer_tag(iCtx, iEp[idx].tag);
        u8 type = 0;
        u8 pl[TGCP_MAX_PAYLOAD];
        u32 plen = sizeof(pl);
        TInt r = tgcp_parse_datagram(iCtx, d, (u32)n, &type, pl, &plen);
        if (r == TGCP_DUP) return;
        if (r != TGCP_OK) {
            iProtoErr++;
            if (iFirstProto < 8) {
                iFirstProto++;
                TBuf8<64> b;
                b.Format(_L8("parse error %d from ep%d len %d\n"), r, idx, n);
                Write(b);
            }
            return;
        }
        if (iLocked < 0) {
            iLocked = idx;
            LogEp(_L8("locked, first packet from"), idx);
            TBuf8<48> b;
            b.Format(_L8("first packet type %d\n"), (TInt)type);
            Write(b);
        }
        if (type == TGCP_PKT_STREAM_DATA || type == TGCP_PKT_STREAM_X2 || type == TGCP_PKT_STREAM_X3) {
            u32 off = 0;
            while (off < plen) {
                u8 sid;
                u32 pts, olen;
                const u8* opus;
                TInt c = tgcp_parse_stream_data(pl + off, plen - off, &sid, &pts, &opus, &olen);
                if (c <= 0) break;
                off += (u32)c;
                iRecvPkts++;
                JbPush(pts, opus, (TInt)olen);
            }
        }
        Pump();
    }
    // sorted insert into the jitter buffer (60 ms units)
    void JbPush(TUint32 aPts, const TUint8* aData, TInt aLen)
    {
        if (aLen <= 0 || aLen > KJbPkt) return;
        if (iHavePts && (TInt)(aPts - iNextPts) < 0) { iLate++; return; }
        TInt i = 0;
        while (i < iJbCount && (TInt)(iJb[i].pts - aPts) < 0) i++;
        if (i < iJbCount && iJb[i].pts == aPts) return;      /* duplicate */
        if (iJbCount == KJbSlots) {                         /* drop the oldest */
            JbShift();
            if (i > 0) i--;
        }
        for (TInt k = iJbCount; k > i; k--) iJb[k] = iJb[k - 1];
        iJb[i].pts = aPts;
        iJb[i].len = aLen;
        Mem::Copy(iJb[i].data, aData, aLen);
        iJbCount++;
        iEverRecv = ETrue;
    }
    void JbShift()
    {
        for (TInt k = 1; k < iJbCount; k++) iJb[k - 1] = iJb[k];
        iJbCount--;
    }
    void NextPacket()
    {
        Mem::FillZ(iPcm60, sizeof(iPcm60));
        if (!iDec) return;
        if (!iPlaying) {
            if (iJbCount < 2) return;                       /* prefill 2 packets */
            iPlaying = ETrue;
            iHavePts = EFalse;
            iPlc = 0;
        }
        TInt t0 = NowUs();
        TInt r = 0;
        TBool plc = EFalse;
        while (iJbCount > 0 && iHavePts && (TInt)(iJb[0].pts - iNextPts) < 0) {
            JbShift();                                      /* stale */
            iLate++;
        }
        if (iJbCount > 0) {
            TInt gap = iHavePts ? (TInt)(iJb[0].pts - iNextPts) : 0;
            if (gap >= TGCP_FRAME_MS && gap <= 600) {
                plc = ETrue;                                /* a packet is missing */
                iNextPts += TGCP_FRAME_MS;
            } else {
                r = opus_decode(iDec, iJb[0].data, iJb[0].len, iPcm60, KRelayFrame, 0);
                iNextPts = iJb[0].pts + TGCP_FRAME_MS;
                iHavePts = ETrue;
                JbShift();
                iPlc = 0;
            }
        } else {
            iUnderruns++;
            iPlc++;
            plc = iEverRecv;
            if (iHavePts) iNextPts += TGCP_FRAME_MS;
            if (iPlc >= 5) iPlaying = EFalse;               /* rebuild the prefill */
        }
        if (plc) {
            r = opus_decode(iDec, NULL, 0, iPcm60, KRelayFrame, 0);
            iPlcFrames++;
        }
        iDecUs += NowUs() - t0;
        if (r > 0) iDecFrames++;
        else if (r < 0) Check(_L8("Decode"), r);
    }
    TBool Connected() { return iRelayOk && iCtx && tgcp_get_state(iCtx) == TGCP_ST_CONNECTED; }
    // build and send whatever the protocol core wants to send now
    void Pump()
    {
        if (!iRelayOk) return;
        TUint32 now = NowMs();
        for (TInt k = 0; k < 6; k++) {
            u8 out[TGCP_MAX_DATAGRAM + 32];
            TInt r = tgcp_tick(iCtx, now, out, sizeof(out));
            if (r <= 0) {
                if (r < 0) Check(_L8("Tick"), r);
                break;
            }
            if (iLocked >= 0) {
                SendTo(iRelaySock, iEp[iLocked].addr, out, r);
            } else {
                // not decided yet: this is the INIT, same datagram to every relay
                for (TInt e = 0; e < iNEp; e++) {
                    tgcp_retag(out, iEp[e].tag);
                    SendTo(iRelaySock, iEp[e].addr, out, r);
                }
                iInitTx++;
            }
        }
    }
    void SendPings(TUint32 aNow)
    {
        iPingsSent++;
        iLastPing = aNow;
        for (TInt e = 0; e < iNEp; e++) {
            u8 q[8];
            Mem::FillZ(q, 8);
            q[0] = (u8)e;
            q[1] = (u8)iPingsSent;
            q[4] = (u8)(aNow & 255);
            q[5] = (u8)((aNow >> 8) & 255);
            u8 out[48];
            TInt r = tgcp_make_reflector_ping(iEp[e].tag, q, out, sizeof(out));
            iPingMs[e] = aNow;
            if (r > 0) SendTo(iRelaySock, iEp[e].addr, out, r);
        }
    }
    static TInt Tick20(TAny* aSelf)
    {
        ((CRunner*)aSelf)->DoTick20();
        return 0;
    }
    void DoTick20()
    {
        if (!iRelayOk) return;
        if (iRelayRecv && !iRelayRecv->IsActive()) iRelayRecv->Start(iRelaySock);
        if (iV277) { Tick277(); return; }
        TUint32 now = NowMs();
        if (iLocked < 0 && iPingsSent < KPingMax && (!iPingsSent || (TInt)(now - iLastPing) >= 500))
            SendPings(now);
        Pump();
        const TDesC8* name = &iStatusName;
        TInt st = tgcp_get_state(iCtx);
        TPtrC8 want(_L8("connecting"));
        if (st == TGCP_ST_FAILED) want.Set(_L8("failed"));
        else if (st == TGCP_ST_CONNECTED) want.Set(tgcp_is_reconnecting(iCtx) ? _L8("reconnecting") : _L8("connected"));
        if (iStatusName != want) {
            iStatusName.Copy(want);
            TBuf8<64> b;
            b.Format(_L8("state %S (core %d fail_reason %d)\n"), name, st, iCtx->fail_reason);
            Write(b);
            SendStatus(iStatusName);
        }
    }
    // ---- MVoIPDownlinkObserver
    void FillBuffer(const CVoIPAudioDownlinkStream&, CVoIPDataBuffer* aBuffer)
    {
        TPtr8 p(NULL, 0);
        aBuffer->GetPayloadPtr(p);
        TInt n = p.MaxLength() < KFrameBytes ? (p.MaxLength() & ~1) : KFrameBytes;
        p.SetLength(n);
        TInt16* pcm = (TInt16*)p.Ptr();
        TInt samples = n / 2;
        if (samples > KFrameSamples) samples = KFrameSamples;
        Mem::FillZ(pcm, n);
        if (iV277) {
            if (!iDnPlaying && iDnCount >= KDnPrefill) iDnPlaying = ETrue;
            if (iDnPlaying) {
                if (iDnCount >= KFrameSamples) {
                    for (TInt i = 0; i < samples; i++) pcm[i] = iDnPcm[(iDnHead + i) % KDnRing];
                    iDnHead = (iDnHead + KFrameSamples) % KDnRing;
                    iDnCount -= KFrameSamples;
                    iUnderrun = 0;
                } else {
                    iUnderruns++;
                    if (++iUnderrun >= 5) iDnPlaying = EFalse;   /* rebuild the prefill */
                }
            }
        } else if (iRelayMode) {
            if (Connected()) {
                if (iPcmPos >= KRelayFrame) { NextPacket(); iPcmPos = 0; }
                Mem::Copy(pcm, iPcm60 + iPcmPos, samples * 2);
                iPcmPos += KFrameSamples;
            } else {
                iPcmPos = KRelayFrame;
            }
        } else if (!iPlaying && iRingCount >= KPrefill) { iPlaying = ETrue; iPlc = 0; }
        if (!iRelayMode && iPlaying && iDec) {
            TInt t0 = NowUs();
            TInt r;
            if (iRingCount > 0) {
                r = opus_decode(iDec, iRing + iRingHead * KMaxPkt, iRingLen[iRingHead],
                                pcm, KFrameSamples, 0);
                iRingHead = (iRingHead + 1) % KRingSize;
                iRingCount--;
                iPlc = 0;
            } else {
                iUnderruns++;
                iPlc++;
                if (iEverRecv) {
                    r = opus_decode(iDec, NULL, 0, pcm, KFrameSamples, 0);
                    iPlcFrames++;
                } else {
                    r = 0;
                }
                if (iPlc >= 5) iPlaying = EFalse; // rebuild the prefill
            }
            iDecUs += NowUs() - t0;
            if (r > 0) iDecFrames++;
            else if (r < 0) Check(_L8("Decode"), r);
        }
        for (TInt i = 0; i < samples; i++) {
            TInt a = pcm[i] < 0 ? -pcm[i] : pcm[i];
            if (a > iPeakOut) iPeakOut = a;
        }
        iPlayBuf->SetPayloadPtr(p);
        TInt err = iDn->BufferFilled(iPlayBuf);
        if (err != KErrNone) Check(_L8("BufferFilled"), err);
    }
    void Event(const CVoIPAudioDownlinkStream&, TInt aType, TInt aErr)
    {
        if (aType == MVoIPDownlinkObserver::KOpenComplete) {
            if (aErr == KErrNone) iDnOpen = ETrue; else Check(_L8("DnOpenComplete"), aErr);
            MaybeStart();
        } else if (aType == MVoIPDownlinkObserver::KDownlinkError) {
            Check(_L8("DnError"), aErr ? aErr : -1);
        } else if (aType == MVoIPDownlinkObserver::KAudioDeviceChanged) {
            if (iStarted) ReportRoute();
        }
    }
    // ---- MVoIPUplinkObserver
    void EmptyBuffer(const CVoIPAudioUplinkStream&, CVoIPDataBuffer* aBuffer)
    {
        TPtr8 p(NULL, 0);
        aBuffer->GetPayloadPtr(p);
        TInt n = p.Length() & ~1;
        const TInt16* s = (const TInt16*)p.Ptr();
        for (TInt i = 0; i < n / 2; i++) {
            TInt a = s[i] < 0 ? -s[i] : s[i];
            if (a > iPeakIn) iPeakIn = a;
        }
        if (iV277) {
            if (n >= KFrameBytes && iEnc && Media277() && !iMuted) {
                Mem::Copy(iUpAcc + iUpN * KFrameSamples, s, KFrameBytes);
                iUpN++;
                if (iUpN >= 3) {
                    iUpN = 0;
                    TUint8 pkt[KJbPkt];
                    TInt t0 = NowUs();
                    TInt r = opus_encode(iEnc, iUpAcc, KRelayFrame, pkt, KJbPkt);
                    iEncUs += NowUs() - t0;
                    if (r > 0) {
                        iEncFrames++;
                        if (c277_session_send_opus(iS, pkt, (u32)r, NowMs()) == C277_SESS_OK) iSentPkts++;
                        Drain277();
                    } else {
                        Check(_L8("Encode"), r ? r : -1);
                    }
                }
            } else {
                iUpN = 0;
            }
        } else if (iRelayMode) {
            if (n >= KFrameBytes && iEnc && Connected() && !iMuted) {
                Mem::Copy(iUpAcc + iUpN * KFrameSamples, s, KFrameBytes);
                iUpN++;
                if (iUpN >= 3) {
                    iUpN = 0;
                    TUint8 pkt[KJbPkt];
                    TInt t0 = NowUs();
                    TInt r = opus_encode(iEnc, iUpAcc, KRelayFrame, pkt, KJbPkt);
                    iEncUs += NowUs() - t0;
                    if (r > 0) {
                        iEncFrames++;
                        u8 out[TGCP_MAX_DATAGRAM + 32];
                        TInt dl = tgcp_send_audio(iCtx, pkt, (u32)r, out, sizeof(out));
                        if (dl > 0 && iLocked >= 0) {
                            if (SendTo(iRelaySock, iEp[iLocked].addr, out, dl)) iSentPkts++;
                        } else if (dl < 0) {
                            Check(_L8("SendAudio"), dl);
                        }
                    } else {
                        Check(_L8("Encode"), r ? r : -1);
                    }
                }
            } else {
                iUpN = 0;
            }
        } else if (n >= KFrameBytes && iEnc) {
            TUint8 pkt[KMaxPkt];
            TInt t0 = NowUs();
            TInt r = opus_encode(iEnc, s, KFrameSamples, pkt, KMaxPkt);
            iEncUs += NowUs() - t0;
            if (r > 0) {
                iEncFrames++;
                if (iNetOk) {
                    if (SendTo(iSock, iDest, pkt, r)) iSentPkts++;
                }
            } else {
                Check(_L8("Encode"), r ? r : -1);
            }
        }
        TInt err = iUp->BufferEmptied(aBuffer);
        if (err != KErrNone) Check(_L8("BufferEmptied"), err);
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
    void Event(const CVoIPFormatIntfc&, TInt) {}

    static TInt Tick(TAny* aSelf)
    {
        ((CRunner*)aSelf)->DoTick();
        return 0;
    }
    void DoTick()
    {
        iTicks++;
        TInt avg = iEncFrames > iLastEnc ? iEncUs / (iEncFrames - iLastEnc) : 0;
        TBuf8<320> line;
        if (iV277 && iS) {
            line.Format(_L8("r %d ver 277 ice %d refl %d sig_acked %d media %d sent %d recv %d lost %d "
                            "plc %d underruns %d sigq %d errors %d last %d\n"),
                        iTicks, c277_session_ice_state(iS), c277_session_refl_ready(iS),
                        c277_session_signaling_acked(iS), c277_session_media_flowing(iS, NowMs()),
                        iSentPkts, iRecvPkts, iLostPkts, iPlcFrames, iUnderruns, iSigOut,
                        iErrors, iLast);
            Write(line);
            if (iVideoEver && iS) {
                TBuf8<400> vl;
                vl.Format(_L8("v %d on %d fps10 %d bps %d au %d pk %d idr %d drop %d units %d pli %d fir %d nack %d "
                              "twcc %d remb %d keyreq %d peer_vid %d peer_rtp %d rr_frac %d rr_lost %d\n"),
                          iTicks, c277_session_video_active(iS) ? 1 : 0, (TInt)iS->vtx_last_fps, (TInt)iS->vtx_last_bps,
                          (TInt)iS->vtx.n_au, (TInt)iS->vtx.n_pk, (TInt)iS->vtx.n_idr, (TInt)iS->vtx.n_drop, iVUnits,
                          (TInt)iS->pli_n, (TInt)iS->fir_n, (TInt)iS->nack_n, (TInt)iS->twcc_n, (TInt)iS->remb_n,
                          iKeyReqs, (TInt)iS->peer_vid, (TInt)iS->n_peer_vrtp,
                          iS->rr_have ? (TInt)iS->rr_fraction : 0, iS->rr_have ? (TInt)iS->rr_lost : 0);
                Write(vl);
                // peer's RTCP receiver report about our video: loss fraction (0..255), for the adaptive bitrate
                if (c277_session_video_active(iS) && iS->rr_have) SendInfo('L', (TInt)iS->rr_fraction);
            }
            if (iRxWant && iS->vrx.started) {
                TBuf8<300> rl;
                rl.Format(_L8("rx %d pk %d au %d idr %d lost %d dropau %d late %d waitau %d pli %d rr %d rtx %d fwd %d sent %d ssrc %d reord %d gapto %d nack %d nofwd %d\n"),
                          iTicks, (TInt)iS->vrx.n_pk, (TInt)iS->vrx.n_au, (TInt)iS->vrx.n_idr, (TInt)iS->vrx.n_lost,
                          (TInt)iS->vrx.n_drop_au, (TInt)iS->vrx.n_late, (TInt)iS->vrx.n_wait, (TInt)iS->rx_pli_sent,
                          (TInt)iS->rx_rr_sent, (TInt)iS->rx_nack_rtx, iRxFwd, iRxAu, (TInt)iS->vrx.ssrc,
                          (TInt)iS->vrx.n_reordered, (TInt)iS->vrx.n_gap_timeout, (TInt)iS->rx_nack_sent, (TInt)iS->rx_nofwd);
                Write(rl);
            }
            if (iNetOk && iStatusName.Length()) SendStatus(iStatusName);
            if (iNetOk) SendStatus(_L8("ready"));
            if (iTicks >= iSeconds) CActiveScheduler::Stop();
            return;
        }
        if (iRelayMode) {
            TInt rtt = iLocked >= 0 ? iRtt[iLocked] : 0;
            if (iLocked < 0) for (TInt i = 0; i < iNEp; i++) if (iPongSeen[i] && (!rtt || iRtt[i] < rtt)) rtt = iRtt[i];
            line.Format(_L8("r %d state %S sent %d recv %d acks_unacked %d lost %d plc %d underruns %d "
                            "reflector_rx %d init_tx %d rtt_ms %d mic %d spk %d errors %d last %d\n"),
                        iTicks, &iStatusName, iSentPkts, iRecvPkts,
                        iCtx ? (TInt)tgcp_unacked_out(iCtx) : 0, iCtx ? (TInt)iCtx->lost_out : 0,
                        iPlcFrames, iUnderruns, iReflectorRx, iInitTx, rtt, iPeakIn, iPeakOut,
                        iErrors, iLast);
            iPeakIn = 0;
            iPeakOut = 0;
            Write(line);
            if (iNetOk && iStatusName.Length()) SendStatus(iStatusName);   /* resend: cheap, idempotent */
            if (iTicks >= iSeconds) CActiveScheduler::Stop();
            return;
        }
        line.Format(_L8("t %d enc_frames %d sent_pkts %d dec_frames %d recv_pkts %d plc %d "
                        "underruns %d jitter_depth %d enc_us_avg %d mic_peak %d spk_peak %d "
                        "errors %d last %d\n"),
                    iTicks, iEncFrames, iSentPkts, iDecFrames, iRecvPkts, iPlcFrames,
                    iUnderruns, iRingCount, avg, iPeakIn, iPeakOut, iErrors, iLast);
        iLastEnc = iEncFrames;
        iEncUs = 0;
        iPeakIn = 0;
        iPeakOut = 0;
        Write(line);
        if (iTicks >= iSeconds) CActiveScheduler::Stop();
    }
    void Write(const TDesC8& aText)
    {
        iFile.Write(aText);
        iFile.Flush();
    }
    void FinalLog()
    {
        if (iRelayMode) Write(_L8("state ended\n"));
    }
private:
    struct TEp { TInetAddr addr; TUint8 tag[16]; };
    struct TJb { TUint32 pts; TInt len; TUint8 data[KJbPkt]; };
    TInt iSeconds;
    TBool iRelayMode;
    CPeriodic* iTimer20;
    CRecv* iRelayRecv;
    RSocket iRelaySock;
    RConnection iConn;
    TBool iConnOk, iRelayOk;
    CSend* iSends[KSends];
    tgcp_ctx* iCtx;
    TUint8 iKey[256];
    TInt iOut;
    TInt iIap;
    TEp iEp[KMaxEp];
    TInt iNEp, iLocked;
    TUint32 iPingMs[KMaxEp];
    TInt iRtt[KMaxEp];
    TBool iPongSeen[KMaxEp];
    TInt iPingsSent;
    TUint32 iLastPing;
    TBuf8<16> iStatusName;
    TInt iReflectorRx, iInitTx, iProtoErr, iFirstProto, iLate;
    TJb iJb[KJbSlots];
    TInt iJbCount;
    TBool iHavePts;
    TUint32 iNextPts;
    TInt16 iPcm60[KRelayFrame];
    TInt iPcmPos;
    TInt16 iUpAcc[KRelayFrame];
    TInt iUpN;
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
    RSocketServ iSs;
    RSocket iSock;
    TInetAddr iDest;
    CRecv* iRecv;
    OpusEncoder* iEnc;
    OpusDecoder* iDec;
    TUint8* iRing;
    TInt iRingLen[KRingSize];
    TInt iRingHead, iRingCount;
    TBool iDnOpen, iUpOpen, iStarted, iNetOk;
    TBool iMuted;
    TInt iWantRoute;
    TBool iPlaying, iEverRecv;
    TInt iPlc;
    TInt iErrors, iLast, iFirstErrs;
    TInt iEncFrames, iSentPkts, iDecFrames, iRecvPkts;
    TInt iPlcFrames, iUnderruns, iEncUs, iDecUs, iLastEnc;
    TInt iPeakIn, iPeakOut;
    // v277
    TBool iV277;
    c277_session* iS;
    TBool iCap;
    c277_sess_conn iConn277[KMaxConn];
    TInt iNConn;
    TUint8 iRnd[56];
    TInt16* iDnPcm;
    TInt iDnHead, iDnCount, iUnderrun;
    TBool iDnPlaying;
    TInt iLostPkts, iUdpTx, iSigOut, iSigIn, iLastIce;
    TBool iEvCreds, iEvCand, iEvRefl, iEvRtp, iEverConn;
    // video send
    RSocket iVSock;
    CVRecv* iVRecv;
    TBool iVNet, iVideoEver;
    TInt iRxWant, iRxFwd, iRxId, iRxAu;
    TInetAddr iTgvRx;
    TInetAddr iTgv;
    TInt iVUnits, iKeyReqs, iLastPeerVid, iLastReqVid, iLastVidAct;
};

void CRecv::RunL()
{
    if (iOwner.Received(iKind, iStatus.Int(), iPtr, iFrom)) Start(*iSock);
}

void CVRecv::RunL()
{
    TInt e = iStatus.Int();
    if (e == KErrNone) iOwner.VideoUnit(iPtr);
    if (e != KErrCancel && e != KErrAbort) Start(*iSock);
}

static void RunL()
{
    TInt seconds = 60;
    TBool relay = EFalse;
    TBuf<32> cmd;
    if (User::CommandLineLength() <= cmd.MaxLength()) {
        User::CommandLine(cmd);
        TLex lex(cmd);
        lex.SkipSpace();
        if (cmd.Left(5) == _L("relay")) {
            relay = ETrue;
            lex.Inc(5);
            lex.SkipSpace();
        }
        TInt v;
        if (lex.Val(v) == KErrNone && v > 0 && v <= (relay ? 7200 : 3600))
            seconds = v;
        else if (relay)
            seconds = 7200;
    }
    CActiveScheduler* sched = new (ELeave) CActiveScheduler;
    CleanupStack::PushL(sched);
    CActiveScheduler::Install(sched);
    CRunner* r = new (ELeave) CRunner(seconds, relay);
    CleanupStack::PushL(r);
    TRAPD(err, r->StartL());
    if (err == KErrNone)
        CActiveScheduler::Start();
    r->FinalStatus();
    r->FinalLog();
    CleanupStack::PopAndDestroy(2, sched);
    User::LeaveIfError(err);
}

GLDEF_C TInt E32Main()
{
    CTrapCleanup* cleanup = CTrapCleanup::New();
    TRAPD(err, RunL());
    delete cleanup;
    CloseSTDLIB();
    return err;
}
