# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A [MakeCode (PXT)](https://makecode.microbit.org/) extension for micro:bit that provides a LoRa radio driver for the HopeRF RFM95W / Semtech SX127x chipset over SPI. The entire driver lives in `rfm95w.ts` under the `Lora` namespace.

## Dev setup

After cloning, two one-time steps are needed:

```sh
makecode install   # populates pxt_modules/ with the core micro:bit types
npm install        # installs TypeScript for local type-checking
```

`makecode` is installed globally via npm (`npm install -g makecode`). `pxt_modules/`, `node_modules/`, and `built/` are all gitignored.

### Type-checking

```sh
npx tsc --noEmit
```

Do not pass a filename (`tsc --noEmit rfm95w.ts`) — that makes tsc ignore `tsconfig.json` and lose the `pxt_modules` include path.

The tsconfig picks up `pxt_modules/**/*.ts` via `**/*.ts`. The core module's `pxt-core.d.ts` carries `/// <reference no-default-lib="true"/>`, which tells TypeScript to suppress its own standard library and use PXT's definitions instead — so no `noLib` flag is needed.

### Build

```sh
makecode build     # or just: makecode
makecode build -w  # watch mode
```

CI runs `npx makecode` on every push via `.github/workflows/makecode.yml`. There is no separate test suite — validation is done by building with MakeCode's toolchain.

## PXT extension conventions

- `//% block="..."` annotations define how a function appears in the MakeCode block editor.
- `//% group="..."` organises blocks into drawer groups (Configuration, Send, Receive).
- `//% advanced=true` hides a block under the "more" section by default.
- `//% weight=N` controls ordering within a group (higher = earlier).
- Enums with `//% block="..."` on each member become drop-down menus.
- `pxt.json` declares the extension metadata; `files` must list every source file explicitly.

## Architecture

The driver is polling-based — no interrupt pin is required. Both TX and RX are driven by `pauseUntil()` loops against `REG_IRQ_FLAGS`.

Key design points:
- **SPI**: 1 MHz, 8-bit mode 0. NSS is bit-banged (P16). Registers are read/written via `readReg`/`writeReg`; bulk transfers via `burstRead`/`burstWrite`.
- **State machine**: radio stays in `MODE_RX_CONTINUOUS` except during transmit. After TX completes, `sendBuffer` polls for `IRQ_TX_DONE`, clears the flag, and re-enters RX before returning.
- **RX fiber**: `init()` spawns a `control.runInBackground` loop that uses `pauseUntil(() => IRQ_RX_DONE)`, reads the FIFO, and dispatches the user callback via a second background fiber.
- **LDRO**: `applyLdro()` is called after any SF or BW change; it enables Low Data Rate Optimize when symbol duration exceeds 16 ms (required by the SX127x datasheet).
- **TX power**: PA_BOOST pin path is used throughout. 20 dBm requires writing `0x87` to `REG_PA_DAC`; all other levels use `0x84`.

## Adafruit RFM95W / Semtech SX1276 documentation

- **Adafruit learn guide** (pinouts, wiring, antenna options): https://learn.adafruit.com/adafruit-rfm69hcw-and-rfm96-rfm95-rfm98-lora-packet-padio-breakouts/overview
- **Adafruit product page**: https://www.adafruit.com/product/3072
- **Semtech SX1276 product page** (datasheet, application notes, register map): https://www.semtech.com/products/wireless-rf/lora-connect/sx1276

## PXT / micro:bit API documentation

- **Block/function reference**: https://makecode.microbit.org/reference — all built-in namespaces (`pins`, `control`, `basic`, `serial`, etc.) with parameters and return types.
- **Hardware & device docs**: https://makecode.microbit.org/device — pin layout, SPI/I2C/UART details, memory map, and other hardware specifics.

## Version control

This repo uses **jujutsu (`jj`)** rather than plain git. The `.jj/` directory contains the working copy and op log. Use `jj` commands for status, diff, commits, and history.
