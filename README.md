# Loco-CAN library

The library provides global configurations and definitions for all parts of the project. Especially the ids for the communication are defined here.

## Host test environment

Arduino boards are not required to check protocol changes. A virtual CAN bus, protocol codecs, and canned train scenarios live in [`host/`](host/README.md) and run on a PC:

```bash
make test
build/host/loco-sim --scenario all
```
