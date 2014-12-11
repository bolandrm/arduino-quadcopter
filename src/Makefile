ARDUINO_DIR   = /Applications/Arduino.app/Contents/Resources/Java
USER_LIB_PATH = ./../libraries
OBJDIR        = ./.build-$(BOARD_TAG)
ARDUINO_LIBS  = PinChangeInt PID_v1 Wire MPULib MedianFilter

#BOARD_TAG         = mega2560
BOARD_TAG         = pro5v328
MONITOR_BAUDRATE  = 115200
#MONITOR_PORT      = /dev/tty.usbmodem*
MONITOR_PORT      = /dev/tty.usbserial
AVRDDUDE          = /usr/local/bin/avrdude

include ~/hardware/Arduino-Makefile/Arduino.mk
