# Changelog

# 1.0.0 (2026-10-19)


* feat!: drive gnss-m2sdr's TDM correlator gateware ([567582e](https://github.com/JuliaGNSS/GNSSM2SDR.jl/commit/567582ef4997f85b55d3f7ed4531bf3ddd4bd7b4))


### Bug Fixes

* apply streaming NCO updates via immediate CSRs ([967a468](https://github.com/JuliaGNSS/GNSSM2SDR.jl/commit/967a46813c87b58dac7825529caa35d1e0c02b64))
* **driver:** a dump of a pending assignment confirms its start ([d9d0be3](https://github.com/JuliaGNSS/GNSSM2SDR.jl/commit/d9d0be310470614da1a66fa47cdb764194086ada))
* drop re-delivered DMA records (duplicate dumps double-step the loops) ([46d10d6](https://github.com/JuliaGNSS/GNSSM2SDR.jl/commit/46d10d6aef996da0fe0bdba013ab0ffce9ffb85f))
* drop stale NCO updates instead of committing them ([a0cf7ef](https://github.com/JuliaGNSS/GNSSM2SDR.jl/commit/a0cf7ef8ef4c917283c5582e015212fb7cd3dceb))
* **emulator:** follow gnss-m2sdr[#44](https://github.com/JuliaGNSS/GNSSM2SDR.jl/issues/44)'s review fixes ([ee460da](https://github.com/JuliaGNSS/GNSSM2SDR.jl/commit/ee460da43dbe2a972f422389cace0a2302c2f106))
* enter GC-safe around the blocking DMA1 read ([08fd94c](https://github.com/JuliaGNSS/GNSSM2SDR.jl/commit/08fd94c46bd1bbe6429f4f877d69557274b16468))
* **example:** give examples/ its own environment ([24e2c7a](https://github.com/JuliaGNSS/GNSSM2SDR.jl/commit/24e2c7ab8a3c7db29d2fdc69320ddf466bdc472f))
* **gnss_loop:** reject an unknown --signals list and an out-of-range --core ([dda7df2](https://github.com/JuliaGNSS/GNSSM2SDR.jl/commit/dda7df2707e5484b49e49000e34d32d37640bcf7))
* hold NCO words until their scheduled sample instead of applying on arrival ([30d5de9](https://github.com/JuliaGNSS/GNSSM2SDR.jl/commit/30d5de9214d198f6f76b3c82dc99d3f0983f52ec)), closes [JuliaGNSS/GNSSReceiver.jl#129](https://github.com/JuliaGNSS/GNSSReceiver.jl/issues/129)
* keep the real-time paths off Julia's event loop and scheduler ([e483655](https://github.com/JuliaGNSS/GNSSM2SDR.jl/commit/e4836557ea4fe487415d2ca4b3afbbe855cbb797)), closes [GNSSReceiver.jl#107](https://github.com/GNSSReceiver.jl/issues/107)
* NCO commit latency, code-doppler semantics, poller active-mask ([33ea031](https://github.com/JuliaGNSS/GNSSM2SDR.jl/commit/33ea0313ab44d9d3e56f9764001b49a1f2d84cf6))
* **nco:** queue pending NCO words per channel instead of superseding them ([d5ba2f1](https://github.com/JuliaGNSS/GNSSM2SDR.jl/commit/d5ba2f136f1415f945e011da6eed517922c7fe7d))
* never block the device service task on a channel lock ([dba3aa6](https://github.com/JuliaGNSS/GNSSM2SDR.jl/commit/dba3aa6ba6a6ac03b92b5f6585bb7b86a4eda61e))
* never block the receiver's fold on a scheduled handover ([a5e2f54](https://github.com/JuliaGNSS/GNSSM2SDR.jl/commit/a5e2f54fbd373c10642c56c2b0a5a60bfcb78ddc)), closes [GNSSReceiver.jl#107](https://github.com/GNSSReceiver.jl/issues/107)
* non-blocking DMA record reads via O_NONBLOCK poll loop ([30498b4](https://github.com/JuliaGNSS/GNSSM2SDR.jl/commit/30498b44bfda17616e293cf175d10488703e6eef))
* only rewrite the code RAM when the channel PRN changes ([bd1008a](https://github.com/JuliaGNSS/GNSSM2SDR.jl/commit/bd1008aa51c3eac35d71ce4713f3b0fe68d93027))
* park the :dma NCO writer when idle ([b94882b](https://github.com/JuliaGNSS/GNSSM2SDR.jl/commit/b94882bebac447c33aeb50fc8af98b1f63ad3f59))
* poll DMA buffer counts before the blocking read ([9f1e3e9](https://github.com/JuliaGNSS/GNSSM2SDR.jl/commit/9f1e3e9492359cd12abdea1ba8f0d593c2231c13))
* publish a channel's confirmed arm sample; reject dumps and NCO words before it ([3f19ba8](https://github.com/JuliaGNSS/GNSSM2SDR.jl/commit/3f19ba85570bc12d2c9997393a938e4310035825)), closes [GNSSReceiver.jl#129](https://github.com/GNSSReceiver.jl/issues/129)
* **raw:** hold samples back a tag block so a hole of whole blocks is never misnamed ([60b4404](https://github.com/JuliaGNSS/GNSSM2SDR.jl/commit/60b440485e1d7844f437f27acd27712b772a13f3)), closes [hi#nibble](https://github.com/hi/issues/nibble)
* **raw:** queue a second of DMA buffers in the recorder; count resyncs as events ([54bd2ec](https://github.com/JuliaGNSS/GNSSM2SDR.jl/commit/54bd2ec5b2af7d73bbe59ef637f51fd547ebbc49))
* rotate the poller scan origin; dedup NCO commits ([68c989b](https://github.com/JuliaGNSS/GNSSM2SDR.jl/commit/68c989bae6b9a7633344f43a34b0e4fa60c3062f))
* self-heal a stale DMA writer lock (driver keeps it past fd close) ([ffecf06](https://github.com/JuliaGNSS/GNSSM2SDR.jl/commit/ffecf0661dc4a269af84d7fee09c5ffcdf5c41b2))
* start the DMA writer before reading the record stream ([1f96151](https://github.com/JuliaGNSS/GNSSM2SDR.jl/commit/1f96151ab2fe4bd234307f73f5ee742c2337fdea)), closes [JuliaGNSS/GNSSReceiver.jl#107](https://github.com/JuliaGNSS/GNSSReceiver.jl/issues/107)
* yield in the handover armed-wait spin ([1d251e1](https://github.com/JuliaGNSS/GNSSM2SDR.jl/commit/1d251e1f05243661129dbd9e54b06e999b657c46))


### Features

* code-phase anchor, CSR dump polling, origin override ([eac3c1e](https://github.com/JuliaGNSS/GNSSM2SDR.jl/commit/eac3c1ea06d9959a0a211d73161e82276c49596c))
* detect the gateware's channel count from the CSR map ([ec37e32](https://github.com/JuliaGNSS/GNSSM2SDR.jl/commit/ec37e32555d9a20288bedbc0d7b10812fea2ac03))
* drain DMA1 through a recorder process and a large pipe ([64cca24](https://github.com/JuliaGNSS/GNSSM2SDR.jl/commit/64cca2468aa7a56d2c2556c7c8fe1a7bce728e1d)), closes [GNSSReceiver.jl#107](https://github.com/GNSSReceiver.jl/issues/107)
* **examples:** drive the closed-loop examples through HardwareCorrelatorLink ([0ff58dc](https://github.com/JuliaGNSS/GNSSM2SDR.jl/commit/0ff58dcc63280996fded50e10fc25e809d061650)), closes [gnss-m2sdr#39](https://github.com/gnss-m2sdr/issues/39)
* LiteX-M2SDR vendor package for the hardware-correlator interface ([1ad040e](https://github.com/JuliaGNSS/GNSSM2SDR.jl/commit/1ad040ee4ef19bc41f9c78e5e50554c2f10846a5)), closes [#107](https://github.com/JuliaGNSS/GNSSM2SDR.jl/issues/107)
* **loop:** run the tracking loops in a dedicated gnss_loop process ([dbc754b](https://github.com/JuliaGNSS/GNSSM2SDR.jl/commit/dbc754b6d9a584b67b47e05c178ad3ea05f4bf04)), closes [JuliaGNSS/GNSSReceiver.jl#129](https://github.com/JuliaGNSS/GNSSReceiver.jl/issues/129)
* pin the CSR dump poller to a reserved core (GNSS_POLLER_CORE) ([2b19825](https://github.com/JuliaGNSS/GNSSM2SDR.jl/commit/2b198257379c420b74f2174708396df2d4e3e582))
* precompile the receiver-facing interface on M2SDRCorrelator ([2ea82c7](https://github.com/JuliaGNSS/GNSSM2SDR.jl/commit/2ea82c7547884baa1ff8b3d8615fc81551b23d95)), closes [GNSSReceiver.jl#107](https://github.com/GNSSReceiver.jl/issues/107)
* **replica:** program sub-chip BOC/CBOC/TMBOC replicas and five-tap channels ([9d7a0f6](https://github.com/JuliaGNSS/GNSSM2SDR.jl/commit/9d7a0f63e7f99ce0550e58c3652aaa0e75d03298)), closes [gnss-m2sdr#32](https://github.com/gnss-m2sdr/issues/32)
* **signal:** configure signal, code length and rates per hardware channel ([a5bc2e5](https://github.com/JuliaGNSS/GNSSM2SDR.jl/commit/a5bc2e5623dbc2ca970b376fa7181cf7e34b0eff)), closes [#8](https://github.com/JuliaGNSS/GNSSM2SDR.jl/issues/8)
* support GNSSReceiver 4 / Tracking 8, and declare the replica amplitude ([15568a8](https://github.com/JuliaGNSS/GNSSM2SDR.jl/commit/15568a8127db6a8105f5b76557ab45d69b41bf47)), closes [GNSSReceiver.jl#107](https://github.com/GNSSReceiver.jl/issues/107)


### Reverts

* **raw:** drop the tag-block hold-back; document the long-hole limit instead ([6f59354](https://github.com/JuliaGNSS/GNSSM2SDR.jl/commit/6f593541c5eb992ba0a58dc3e3754db357b5c070))


### BREAKING CHANGES

* the gateware of the old per-channel bank is no longer
supported; M2SDRDriver, M2SDRRemote and the capability mapping are gone.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
