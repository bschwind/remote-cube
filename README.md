# Remote Cube

An ESP32-based project for reading/writing GameCube controller signals and sending them over a network.

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
