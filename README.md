# symbian-s60v3-media-notes

**Knowledge and working examples for camera, H.264 video and full-duplex audio on Symbian S60
3rd Edition FP2 (Nokia N79)**: what the phone's multimedia stack really does, what failed
(with the error codes), which capabilities you need (spoiler: **a hacked phone**), and the
small native programs that were used to find out. It comes out of building voice and video
calls for [RetroTele](https://github.com/manojanasuri16/RetroTele), a Telegram client for the N79.

Read [docs/FINDINGS.md](docs/FINDINGS.md) first; the code is the evidence.

> **You need an unlocked ("hacked") phone for everything that touches the audio or video
> hardware.** All helper programs here are built with `MultimediaDD`, `ReadDeviceData` and
> `WriteDeviceData`, which a self-signed package cannot be granted on a stock S60 3rd Edition
> phone. On the N79 that means EasyHackKit / **RomPatcher+** with **Install Server RP+** and
> **Open4All RP+** (see the [unlock guide](https://github.com/manojanasuri16/symbian-unlock-guide)).
> Verified from the built binaries, see "Capabilities" below.

## Status

- Developed and measured on **one device: a Nokia N79 (S60 3rd Edition FP2, Symbian 9.3)**.
  Other S60 3rd Edition phones, FP1 devices and S60 5th
  Edition are **untested**; the DevVideo codec UIDs below are device specific.
- All six helpers in `native/` were **rebuilt from this repo layout on 2026-11-23** with the
  GnuPoc toolchain (they compile and link). They were not re-run on a phone from this layout, but
  the sources are the ones that produced the results below. `reference/callaudio.cpp` is a read-only
  reference (it needs the call engine and Opus to build).
- The measurements are the numbers recorded in RetroTele's development notes and commit
  messages. The raw probe/benchmark log files lived on the phone's memory card and are not
  included (they contain device identifiers); where the notes do not give a number, none is invented here.

## What is in `native/`

| Program | UID (test range, **change it**) | What it does |
|---|---|---|
| `vidprobe` -> `tgvprobe.exe` | 0xE7A3D5E6 | writes a report of DevVideo decoders/encoders and their profiles, MMF controllers, cameras (formats, frame rates), display; every step is logged before it runs, last line `probe end` |
| `vidbench` -> `tgvbench.exe` | 0xE7A3D5E7 | live H.264 benchmark: encoder alone (several sizes/bitrates, synthetic YUV), decoder alone (embedded 176x144 clip, flat out), camera -> encoder -> decoder, CPU headroom from the idle thread; an encoder initialisation sweep |
| `tgvideo` -> `tgvideo.exe` | 0xE7A3D5E9 | the pipeline: ECam camera -> H.264 encoder -> decoder -> picture in a **global RChunk** (`tgvideo_frames`, layout in `tgvshared.h`); commands and status over UDP on localhost; can export every encoder unit as a UDP datagram for an RTP packetizer; can decode a peer's H.264 and publish it |
| `tgvdisp` -> `tgvdisp.pyd` | 0xE7A3D5EA | a PyS60 module (needs no special capability) that reads that chunk and draws the YUV picture into a PyS60 `graphics.Image` (YUV->RGB in integer math, optional mirror/decimate) |
| `audio/voiptest.cpp` -> `tgvoiptest.exe` | 0xE7A3D5E1 | full-duplex mic -> speaker echo through **Nokia VoIP Audio Services** (the approach that worked) |
| `audio/audiotest.cpp` + `loopback.cpp` -> `tgaudiotest.exe` | 0xE7A3D5C6 | the same echo test through the standard MMF streams (`CMdaAudioInputStream`/`OutputStream`); kept to document why that does not work |
| `reference/callaudio.cpp` | (0xE7A3D5E3) | the real call helper: VoIP audio <-> Opus <-> UDP, plus the RTP/relay engine. about 1,600 lines, for reading |

The UIDs are the author's test-range UIDs (`0xE7A3D5xx`). **Use your own**: two binaries with the
same UID/SID make the phone refuse the whole installation with only "Unable to install".

## Build

Needs the toolchain from [symbian-s60v3-toolchain](https://github.com/manojanasuri16/symbian-s60v3-toolchain)
(GnuPoc + CSL GCCE 3.4.3 + S60 3.2 FP2 SDK + PyS60 1.4.5 SDK; the Nokia SDK and the API Plug-In Pack
(which contains the VoIP Audio Services headers/libs, `voipaudiointfc.lib`) must be obtained by you).

```sh
SYMBIAN_SDK=/path/to/symbian-sdk sh build.sh      # -> out/*.exe, out/tgvdisp.pyd
```

Packaging into a `.sis` is not part of this repo (the RetroTele build scripts use ensymble and a
self-signed certificate; see the [pys60-toolbox](https://github.com/manojanasuri16/pys60-toolbox)
notes). Nokia's VoIP Audio Services SISX (needed on the phone for `voiptest`) is Nokia's and is
**not** redistributable; take it from the API Plug-In Pack you downloaded.

## Capabilities (verified from the binaries)

Read from the E32 image headers of the shipped RetroTele binaries (security info at offset
0x80/0x88):

| Binary | Capabilities |
|---|---|
| tgvprobe, tgvbench, tgvideo | MultimediaDD, ReadDeviceData, WriteDeviceData, UserEnvironment, LocalServices, NetworkServices, ReadUserData, WriteUserData |
| tgvoiptest, tgaudiotest | MultimediaDD, ReadDeviceData, WriteDeviceData, UserEnvironment |
| tgcallaudio (call helper) | MultimediaDD, ReadDeviceData, WriteDeviceData, UserEnvironment, NetworkServices, LocalServices |
| tgvdisp.pyd, tgnative.pyd (loaded into the Python process) | NetworkServices, LocalServices, ReadUserData, WriteUserData, UserEnvironment (user-grantable only) |

`MultimediaDD` is required to use the audio/video device drivers' fast paths (DevVideo
hardware codecs, VoIP audio). `ReadDeviceData`/`WriteDeviceData` appear because the camera and codec
plugins need them. A process may only load DLLs that have at least its own capabilities, and the **PyS60 1.4.5
interpreter lacks `MultimediaDD`** (that is why the helpers are separate EXEs and talk to Python over
localhost UDP and a shared chunk instead of being `.pyd` modules).

## Licence

MIT (see [LICENSE](LICENSE)). `native/vidbench/clip_data.h` and `native/tgvideo/clip_data.h` are
8 frames of a synthetic test pattern (ffmpeg `testsrc`) encoded with x264, embedded as a byte array
for the decoder test; the bitstream has no personal content. Nokia, Symbian, S60 and ECam/DevVideo
are trademarks or property of their owners; this repo contains no Nokia SDK code, only programs that
`#include` the SDK headers you install yourself.

## Part of the RetroArchive ecosystem

Extracted from [RetroTele](https://github.com/manojanasuri16/RetroTele). All apps, guides and open-source repos are listed on [RetroArchive](https://manojanasuri16.github.io/RetroArchive/) (see the [Open source page](https://manojanasuri16.github.io/RetroArchive/opensource/)); the build toolchain is [symbian-s60v3-toolchain](https://github.com/manojanasuri16/symbian-s60v3-toolchain).
