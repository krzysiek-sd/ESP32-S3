| Supported Targets | ESP32-S3 |
| ----------------- | -------- |

# Test Abort

(See the README.md file in the upper level 'examples' directory for more information about examples.)

This example demonstrates how to cause that processor go to panic abort.

## How to use example

### Hardware Required

The example can be run on ESP32-S series based development board.

### Setup the Hardware


### Configure the project



### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Example Output

Type some characters in the terminal connected to the external serial interface. As result you should see echo in the same terminal which you used for typing the characters. You can verify if the echo indeed comes from ESP board by
disconnecting either `TxD` or `RxD` pin: no characters will appear when typing.

## Troubleshooting

You are not supposed to see the echo in the terminal which is used for flashing and monitoring, but in the other UART configured through Kconfig can be used.
