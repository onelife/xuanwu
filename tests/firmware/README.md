# Test firmware

Prebuilt images that the integration tests boot. They are committed so the suite
needs no toolchain, but every one of them is reproducible from the sketch beside it.

```
tests/firmware/
├── board.yaml            the matrix: sketch -> board (FQBN) -> chip description
├── build.py              rebuilds everything with the Arduino CLI
├── Blink_m4/             sketch sources, one directory per sketch
├── Blink_uart_m3/        (the Arduino CLI only builds X/X.ino)
├── Blink_mkrzero/
├── Blink_f767zi/
├── stm32f411/            built images: .elf/.bin/.hex
├── sam3x8e/
├── mkrzero/
└── nucleo_f767zi/
```

```console
$ python tests/firmware/build.py --list      # what builds what
$ python tests/firmware/build.py             # rebuild all of it
$ python tests/firmware/build.py sam3x8e     # just the Due
```

The Arduino CLI and its cores are installed in the development image
(`docker/Dockerfile`). `docs/manual.zh-CN.md` walks through the manual route,
including how to add a board.

## Which images the test suite runs

`tests/conftest.py` answers that from `board.yaml`: a board that declares a `chip`
gets its images parametrised into the smoke tests, and a board without one — the
SAMD21 and the STM32F767 so far — is built but deliberately left alone, as the work
list for supporting that part. The load address is only a fallback, because two
unrelated parts (STM32F411 and STM32F767) both run from `0x08000000`.

`tests/conformance/test_firmware_matrix.py` keeps the two sides consistent.

## Hand-written firmware

`stm32f411/fpu_test/` is not built by `build.py`: it is a bare-metal Cortex-M4F
program with its own `build.sh` (needs `arm-none-eabi-gcc`), because it has to
control the FPU and the exception frame itself. It lives under a declared output
directory, which is how the conformance test accounts for it.

`.map` files and the `*.with_bootloader.*` images are build output and are ignored.
