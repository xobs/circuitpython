USB_VID = 0x1209
USB_PID = 0x5BF0
USB_PRODUCT = "Hackaday Supercon 2019 Badge"
USB_MANUFACTURER = "Hackaday"
USB_DEVICES = "CDC,MSC,AUDIO,HID"

INTERNAL_FLASH_FILESYSTEM = 1
LONGINT_IMPL = MPZ

# The default queue depth of 16 overflows on release builds,
# so increase it to 32.
CFLAGS += -DCFG_TUD_TASK_QUEUE_SZ=64

# The HaD badge supports multiply, atomics, and compressed instructions
CFLAGS += -march=rv32imac -mabi=ilp32
LDFLAGS += -march=rv32imac -mabi=ilp32

CIRCUITPY_NEOPIXEL_WRITE = 1
CIRCUITPY_DIGITALIO = 1
CIRCUITPY_MICROCONTROLLER = 1
