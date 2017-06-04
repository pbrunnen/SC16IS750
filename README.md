# SC16IS750
Python UART I2C Interface Library for the NXP SC16IS750

######################################################
#
#  (C) 2 0 1 7,   P e t e r   B r u n n e n g r ä b e r
#
# Inspired by Tiequan Shao's "Sandbox Electronics" C++ code
#   and by Tony DiCola's Adafruit HT16K33 library
#
######################################################

# REFERENCES
#
# http://sandboxelectronics.com/?p=472
# https://github.com/SandboxElectronics/UART_Bridge/blob/master/SC16IS750.cpp
# 

# v0.01.056 NOTES
#
#  - Only I2C is implemented at this point
#  - Xon Any function (MCR[5]) is not implemented
#  - Special character (EFR[5]) is not implemented
#  - Because of I2C use, interrupt support is not implemented
#  - RS485 support needs expanding on EFCR
#  - EFCR Transmit and Receive disable flags not implemented
#  - GPIO interface not yet implemented
#
