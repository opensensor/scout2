# Flight Controller

A high-performance flight controller implementation in C for the Raspberry Pi Pico.

## Building

```bash
mkdir build
cd build
cmake ..
make
```

## Directory Structure

- src/ - Source files
  - core/ - Core flight controller logic
  - drivers/ - Hardware abstraction layer
  - math/ - Math utilities
  - utils/ - Utility functions
- include/ - Public header files
- tests/ - Unit tests
- tools/ - Development tools

## License

MIT License
