# Remote Cube

An ESP32-based project for reading/writing GameCube controller signals and sending them over a network.

**Status:** It works! Use `make menuconfig` to define whether the ESP32 is a "client" (sending controller data) or a "server" (receiving and playing back controller data). The server needs a listening port specified, and the client needs a target IP + port to send to. Currently only UDP is used. This has been tested with a RISC-V ESP32 C3 as the "client" and a plain dual-core ESP32 as the "server".

This project has been confirmed to work on first-party controller adapters. Third-party adapters have given some trouble in the past, they need to be tested again.

## Clone

* Use the `--recursive` flag when cloning to pick up the ESP-IDF

## Dependencies

* ESP-IDF Tools
    * `cd esp-idf`
    * `export IDF_GITHUB_ASSETS="dl.espressif.com/github_assets"`
    * `./install.sh`

## Configure

```bash
$ make menuconfig
```

## Build

```bash
$ make -j8 # Or pick your favorite parallel number
```

## Run (and see serial output)

```bash
$ make -j8 flash monitor
```

## Monitor Serial Output

```bash
$ make monitor
```

Exit with `Ctrl + ]`.

## Erase Flash

```bash
$ make erase_flash
```

## Code Format
-----------

`clang-format` is used for code formatting.

```bash
$ make format
```
