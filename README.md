# RP2040 BSEC Library

The RP2040 BSEC Library provides an interface to the Bosch BSEC (Bosch Software Environmental Cluster) library for use on the Raspberry Pi Pico (RP2040). This library allows you to interact with the BME68x sensor for environmental sensing, including IAQ (Indoor Air Quality), CO₂ levels, VOCs, and more. The library has been tailored for RP2040 boards and uses I2C communication with the BME68x sensor.

## Features

- Provides data from the BME68x sensor through the BSEC algorithm, which includes IAQ, static IAQ, CO₂ equivalent, breath VOC equivalent, temperature, humidity, and more.
- Supports configuration and state management of the BSEC algorithm.
- Allows control over sensor parameters like sample rate and output sensor subscriptions.
- Efficient I2C communication support for embedded systems.

## Installation

### Git

To install the RP2040 BSEC Library, add it as a submodule to your project. Ensure you install the library recursively, as it relies on other submodules.

```sh
git submodule add --recursive https://github.com/your-username/rp2040-bsec.git lib/rp2040-bsec
```

### CMake

After adding the library as a submodule, update your `CMakeLists.txt` to include the RP2040 BSEC Library.

```cmake
# After the `pico_sdk_init` call
add_subdirectory(lib/rp2040-bsec)

# Link the library to your project
target_link_libraries(your_project rp2040_bsec)

# Include the library's header files
target_include_directories(your_project PRIVATE lib/rp2040-bsec)
```

### First Steps

1. **Setup I2C**: Make sure your RP2040 board is connected to the BME68x sensor via I2C. The library currently supports I2C communication exclusively.
2. **Initialization**: Initialize the BSEC library in your code by creating an instance of the `Bsec` class and configuring the sensor and sampling parameters. (see original .ino files for now, examples to be added to the project)
3. **Subscription Configuration**: Configure which data outputs you need and their sample rates through the `updateSubscription` method.
4. **Running the Library**: Call the `run` method to obtain sensor data whenever available.

## Known Limitations

- **I2C Only**: This library supports I2C communication only; SPI is currently not supported.
- **Single Device Support**: Only one BME68x sensor can be used at a time. Multi-device support is planned for future updates.
- **BSEC Version**: The library uses an older version of the BSEC library (v1.4) due to challenges in porting the newer BSEC 2.0, which is tightly integrated with the BME68x Arduino library.
