# API

## GNSSM2SDR: the receiver's side

```@docs
GNSSM2SDR
start_raw_stream
RawStream
device_sample_origin
enable_tagging!
gnss_loop_command
default_loop_executable
```

## M2SDRLoop: the loop process

```@docs
M2SDRLoop
loop_main
```

### Driver

```@docs
TDMDriver
start_device!
stop_device!
```

### Reaching the gateware

```@docs
TDMIO
LitePCIeTDM
TDMEmulator
feed!
TDMRegs
reg_read
reg_write
LiteXCSR
DMAWriterStream
```

### Record format

```@docs
TDMRecord
decode_record
TDMCapabilities
decode_capabilities
tap_layout
```
