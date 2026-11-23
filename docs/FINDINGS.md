# Findings: camera, H.264 and audio on the Nokia N79 (S60 3rd Edition FP2)

Everything below was observed on one Nokia N79 in November 2026 while building Telegram voice and
video calls (RetroTele). "Verified in code" means it is visible in the sources in this repo;
"measured" means it comes from the phone logs recorded in the project's notes at the time (the raw
logs are not included). Values you may want to re-measure are marked.

## 0. Why helper EXEs?

Python for S60 1.4.5's interpreter process has no `MultimediaDD`, and a process can only load DLLs
with at least its own capabilities, so the audio/video work lives in **separate EXEs built with
MultimediaDD** which the Python app starts. They talk to Python through:

| Channel | Used for |
|---|---|
| UDP on 127.0.0.1 (ASCII commands, status lines, binary units) | control and data; ports used by RetroTele: 47001/47002 (call audio: uplink Opus packets to Python / packets to play), 47011 (tgvideo status, once per second), 47012 (tgvideo commands `S w h fps kbps cam`, `C n`, `K`, `E 1|0`, `D`, `Q`), 47014 (encoder units `'V'` + Annex-B to the RTP packetizer), 47015 (peer video access units to the decoder, fragmented in 1,200-byte pieces) |
| a global `RChunk` named `tgvideo_frames` | decoded and preview pictures: `tgvideo.exe` creates it (`CreateGlobal`), `tgvdisp.pyd` opens it (`OpenGlobal`); a seqlock (odd while writing) guards each buffer; layout in `tgvshared.h` |
| small text config files on the card (deleted after reading when they hold keys) | start-up parameters |

UDP on loopback was chosen over a shared chunk for the video units because the units arrive as events
in the helper's active scheduler (no polling timer, no seqlock, no extra thread); a unit is at most
~10 KB at 15 fps / 60 kbit/s.

## 1. Capabilities and the hacked phone

See the README table, derived from the E32 image headers. Consequences:

- An `.sis` containing a `MultimediaDD`/`ReadDeviceData`/`WriteDeviceData` binary, self-signed, is
  refused on a stock phone ("Update error", "requested application access not granted"). On a
  phone patched with RomPatcher+ (Install Server RP+, **Open4All RP+**) it installs.
- If an install of such a package fails, first check that Open4All is actually switched on.
- A pyd loaded by the Python process (`tgvdisp.pyd`) must stay within the app's own, user-grantable
  capabilities.
- UIDs: use unique `0xE7A3D5xx`-style test-range SIDs for every EXE/DLL. A SID clash (e.g. two
  helpers sharing one) gives a bare "Unable to install".

## 2. Audio

**The standard MMF streams cannot do a full-duplex call on this phone.**

- `CMdaAudioInputStream` recorded fine in the app process (about 33 KB/s at 16 kHz mono PCM16), but every
  `CMdaAudioOutputStream` write while recording failed with **`KErrAccessDenied` (-21)** (comment in
  `loopback.cpp`). Opening the speaker first and keeping it busy with silence, with `EMdaPriorityMax` on both, made writes work.
- In the Python process the microphone then read silence (peak level 69) while the `MultimediaDD`
  helper (`tgaudiotest.exe`, same code) heard real audio (peak 8,540): the app process is not allowed the real mic path.
- Even then the MMF streams are **not full duplex**: the microphone is muted while playback runs.
- A 0.5 s beep before the echo and `CAudioOutput::SetAudioOutputL(EPublic)` (Audio Routing API from the FP2
  API Plug-In Pack) are useful to check the speaker and force the loudspeaker.

**Nokia VoIP Audio Services (VAS) works, and is what to use** (`voiptest.cpp`):

- Needs the system component *VoIPAudioSrvc* installed (a SISX from Nokia's S60 3rd FP2 API Plug-In Pack;
  not redistributable) and the headers/lib `voipaudiointfc.lib`.
- Format: **PCM16, 8 kHz, mono, 20 ms = 160 samples = 320 bytes** per buffer (assumed and verified on this device).
- Uplink gain: **the maximum is 1** on this device; setting "3/4 of max" rounds to **0 and mutes the mic**
  (all-zero samples). Set the maximum.
- In `FillBuffer` fill exactly one 320-byte frame and hand the **factory buffer** back to `BufferFilled`;
  the downlink asks for one buffer only unless you keep supplying them.
- Downlink device can be switched at run time: loudspeaker, handset, or "no preference" (which selected the
  Bluetooth headset when connected).
- Full-duplex echo with it worked (call helper, 2026-11-22). A 120 ms jitter-buffer prefill and 32 KB UDP
  buffers fixed choppy playback over a localhost loop (56 underruns in 20 s at a 3-frame prefill).

## 3. Video hardware: the probe (`vidprobe`)

From the probe report on the N79 (summary as recorded; capabilities and numbers are what DevVideo reported):

| Item | Value |
|---|---|
| H.264 **decoder, hardware** (TI DM299), UID **0x10204bfc** | up to CIF 352x288 @ 30 fps, baseline, profile-level-ids `42800a`..`428014`, `42900b`; `42E0xx` (constrained baseline) not listed; `directdisplay = 0` |
| H.264 decoder, software, UID **0x102073ef** | used as the loopback / peer-video decoder; decodes **QCIF** in practice (the code skips it above 176x144) |
| H.264 **encoder**, "ARM H264 Video Encoder" (Emuzed), UID **0x20001c13** | `accelerated = 0`: **software**. Baseline `42800A`..`42801E`, QVGA/CIF/VGA listed up to 30 fps |
| MPEG-4 and H.263 | encoder and decoder present |
| Rear camera (index 0) | video capture YUV420 planar 640x480, 352x288, 320x240, 176x144 at 30 or 15 fps |
| Front camera (index 1) | 352x288, 320x240, 176x144 at 15 fps |
| Camera frames actually delivered | about 22 fps in capture tests (viewfinder bitmaps 20-28 fps); 11-14 fps in the later call pipeline |
| RAM free / CPU | ~64 MB free; the CPU reports 192 MHz when idle (frequency scaling) |

## 4. Video benchmark (`vidbench`)

Measured with the benchmark (CPU time from the idle thread):

| Encoder (software) | Result |
|---|---|
| 176x144 @ 15 | 12.5 fps achieved, ~6 % CPU, latency 13 ms |
| 176x144 @ 30 | 21 fps, ~18 % CPU |
| 320x240 @ 15 | 12.8 fps, **~50 % CPU**, latency 44 ms |
| 352x288 @ 15 | 10-12.8 fps, 60-70 % CPU |

| Decoder (hardware, flat out) | Result |
|---|---|
| 176x144 | 135 fps |
| 320x240 | 48 fps |
| CIF | 38-40 fps, ~0 % CPU, latency 12 ms when paced |

Decoder output is YUV with `TYuvSamplingPattern` 8 (`EYuv422Chroma1`, 4:2:2) and layout 4; `tgvdisp.pyd` converts it,
and also the camera preview frames. In a real call with audio running, 320x240 gave only 3-6 fps, so calls
start at QCIF 176x144 with a ladder of quality steps.

## 5. What had to be done to make the codecs initialise (and what failed)

**Encoder (0x20001c13)** - the working recipe found by the sweep in `vidbench` (also in `tgvideo.cpp`):

- coded picture unit (`EDuCodedPicture`), elementary-stream encapsulation (`0x10000`);
- **do not call `SetBufferOptionsL`**; random access on; source memory is constant + real-time;
- set the **rate control after `Initialize`**;
- output is H.264 **Annex B** with SPS/PPS;
- **the target bitrate is only loosely honoured**: the encoder spent ~50 kbit/s whatever the target, so
  sharpness is asked for through the encoder **picture quality** setting (80/65/50 per quality step);
- camera video buffers: **2**. Requesting 4 made `PrepareVideoCaptureL` fail with **-5** (the camera then looked unavailable);
- IDR roughly every 2.5-5 s plus on PLI/FIR requests.

**Decoder / camera exclusivity**:

- The **hardware decoder cannot run while the camera captures**: DevVideo then fails with
  **`KErrHardwareNotAvailable` (-44)** ("decoder unavailable while the camera captures (-44)", `tgvideo.cpp`).
  `tgvideo.exe` has an "order sweep" command (`D`) that tries start orders (camera first / decoder first /
  decoder first + viewfinder) to find one that works. The default became: camera + encoder first, then the
  **software** decoder 0x102073ef.
- The hardware decoder **0x10204bfc ends in a fatal error with a peer's High-profile stream and then
  hangs the helper** (a call then never executed later commands), so it is not tried in calls any more
  (set `KRxUidCount` to 2 for experiments).
- Only baseline-family profile-level-ids are listed for the hardware decoder. RetroTele therefore offers only
  H.264 `42e01f` (constrained baseline, packetization-mode 1) in its call signalling, and shows a notice when
  the peer's picture cannot be decoded.

**Error codes seen (standard Symbian values):**

| Code | Name | Where/what |
|---|---|---|
| -5 | `KErrNotSupported` | `PrepareVideoCaptureL` with too many camera buffers (4) |
| -21 | `KErrAccessDenied` | speaker write while the mic records in the MMF streams |
| -44 | `KErrHardwareNotAvailable` | hardware decoder while the camera runs |
| -14 | `KErrInUse` | camera reservation retried by the benchmark |

## 6. Showing video in PyS60

PyS60 1.4.5 has no video surface you can feed. The solution here: `tgvideo.exe` writes the decoded YUV
picture (and a camera preview) into the global chunk; `tgvdisp.pyd` converts it with integer lookup tables
(built on the heap at `attach()`; the module has a single static pointer, hence `EPOCALLOWDLLDATA`) into a
`graphics.Image`, 2x nearest-neighbour when the target is twice the size, optional mirror (front camera
self-view) and decimation. Python then blits the Image to its canvas. API: `attach()`, `detach()`, `info()`,
`render(image, which, flags, last)`; errors -1..-4 are documented at the top of `tgvdisp.cpp`.

## 7. Caveats

- One phone, one firmware. The codec UIDs and limits are what DevVideo on this N79 reported; use
  `vidprobe` to learn your device's.
- `clock()` returns 0 on the phone; time with `User::NTickCount` / `TTime::HomeTime` / the HAL fast counter.
- Write logs with one open per line and always end with a marker line (`probe end`, `bench end`, `mirror end`):
  a hang is then easy to locate: the last line before a missing end marker is the step that hung.
- Logs from these programs contain device and network information: **scrub them before sharing**.
