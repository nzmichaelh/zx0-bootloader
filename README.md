# ZX0 bootloader

This is an I2C based bootloader for the SAMD21. It implements the [Microsoft HF2][hf2] protocol over I2C and uses the same double-tap-for-bootloader and magic as the Adafruit UF2 bootloader.

The bootloader is written in C++ and uses Zephyr for the drivers. It should be easy to adapt to the SAMD51 and other devices supported by Zephyr but each will need a custom I2C client driver.

-- Michael Hope <mlhx@google.com> <michaelh@juju.nz>

[hf2]: https://github.com/microsoft/uf2/blob/master/hf2.md
