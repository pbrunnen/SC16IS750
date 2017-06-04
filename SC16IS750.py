#!/usr/bin/python
# -*- coding: utf-8 -*-
######################################################
#
#   N X P 's   S C 1 6 I S 7 5 0   I 2 C   U A R T
#      B R I D G E   C H I P   L I B R A R Y
#    v0.01.056
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

# NOTES
#
#  - Only I2C is implemented at this point
#  - Xon Any function (MCR[5]) is not implemented
#  - Special character (EFR[5]) is not implemented
#  - Because of I2C use, interrupt support is not implemented
#  - RS485 support needs expanding on EFCR
#  - EFCR Transmit and Receive disable flags not implemented
#  - GPIO interface not yet implemented
#


# ====================================================
#   C O N S T A N T S
# ====================================================


# -- Frequency of the crystal feeding XTAL
# !! Crucial for setting baud rate; must match crystal in use !!
SC16IS750_CRYSTAL_FREQ	= 14745600

# -- General Registers
SC16IS750_REG_RHR		= 0x00	# Receive Holding Register (R)
SC16IS750_REG_THR		= 0x00	# Transmit Holding Register (W)
SC16IS750_REG_IER		= 0x01	# Interrupt Enable Register (R/W)
SC16IS750_REG_FCR		= 0x02	# FIFO Control Register (W)
SC16IS750_REG_IIR		= 0x02	# Interrupt Identification Register (R)
SC16IS750_REG_LCR		= 0x03	# Line Control Register (R/W)
SC16IS750_REG_MCR		= 0x04	# Modem Control Register (R/W)
SC16IS750_REG_LSR		= 0x05	# Line Status Register (R)
SC16IS750_REG_MSR		= 0x06	# Modem Status Register (R)
SC16IS750_REG_SPR		= 0x07	# Scratchpad Register (R/W)
SC16IS750_REG_TCR		= 0x06	# Transmission Control Register (R/W)
SC16IS750_REG_TLR		= 0x07	# Trigger Level Register (R/W)
SC16IS750_REG_TXLVL 	= 0x08	# Transmit FIFO Level Register (R)
SC16IS750_REG_RXLVL 	= 0x09	# Receive FIFO Level Register (R)
SC16IS750_REG_IODIR		= 0x0A	# I/O pin Direction Register (R/W)
SC16IS750_REG_IOSTATE	= 0x0B	# I/O pin States Register (R)
SC16IS750_REG_IOINTENA	= 0x0C	# I/O Interrupt Enable Register (R/W)
SC16IS750_REG_IOCONTROL	= 0x0E	# I/O pins Control Register (R/W)
SC16IS750_REG_EFCR		= 0x0F	# Extra Features Register (R/W)

# -- Special Register Set (Requires LCR[7] = 1 & LCR != 0xBF to use)
SC16IS750_REG_LCR7_DLL	= 0x00	# Divisor Latch LSB (R/W)
SC16IS750_REG_LCR7_DLH	= 0x01	# Divisor Latch MSB (R/W)

# -- Enhanced Register Set (Requires LCR = 0xBF to use)
SC16IS750_REG_LCR_0XBF_EFR		= 0x02	# Enhanced Feature Register (R/W)
SC16IS750_REG_LCR_0XBF_XON1		= 0x04	# XOn Nr.1 Word (R/W)
SC16IS750_REG_LCR_0XBF_XON2		= 0x05	# XOff Nr.1 Word (R/W)
SC16IS750_REG_LCR_0XBF_XOFF1	= 0x06	# XOn Nr.2 Word (R/W)
SC16IS750_REG_LCR_0XBF_XOFF2	= 0x07	# XOff Nr.2 Word (R/W)

# -- Register Bitfield 
SC16IS750_REG_LSR_FIELDS = { 0:"data-in-receiver", 1:"overrun-error", 2:"parity-error", 3:"framing-error", 4:"break-interrupt", 5:"thr-empty", 6:"thr-tsr-empty", 7:"fifo-data-error" }
SC16IS750_REG_MSR_FIELDS = { 0:"cts-delta", 1:"dsr-delta", 2:"ri-delta", 3:"cd-delta", 4:"cts-high", 5:"dsr-high", 6:"ri-high", 7:"cd-high" }





# ====================================================
#   L O A D   L I B R A R I E S
# ====================================================

# Import core python functions
import time
import inspect




# ====================================================
#   S C 1 6 I S 7 5 0   C O M M   I / O
#      C L A S S   D E F I N I T I O N
# ====================================================

class SC16IS750(object):

#
# == Class scope variables ==
#
	_bPrintDebug = False
	_oI2CInstance = None
	_oDeviceInst = None
	_bBaudSet = False
	_bLineSet = False
	_bFlagLockIO = False
	_hRegLCR = 0x00
# -- Determine the sleep millisec. based on one chip cycle by the crystal frequency
	_fSleepMsec = ( ( 1.0 / SC16IS750_CRYSTAL_FREQ ) / 1000.0 )
# -- Timeout must be >2x chip cycles
	_iTimeoutLockIOmsec = ( _fSleepMsec * 10.0 )



#
# == Class Initialization and Setup ==
#
	def __init__(self, hI2CAddress, hI2CBus = 1, _oExistingI2CInstance = None, **kwargs):
	# -- Setup the Adafruit I2C class if a pointer to one is not provided
		if _oExistingI2CInstance is None:
			import Adafruit_GPIO.I2C as oI2C
			self._oI2CInstance = oI2C

	# -- Init the I2C instance for the designated address 
		self._oDeviceInst = self._oI2CInstance.get_i2c_device(hI2CAddress, **kwargs)

	# -- Issue the UART Software Reset...
		self.ResetDevice()

	# -- Always return init without a state
		return



#
# == Enable debug printing ==
#
	def fPrintDebug(self):
		self._bPrintDebug = True
		return




# ----------------------------------------------------
#   C L A S S   I N T E R N A L   C H I P
#      R E G I S T E R   I / O   F U N C T I O N S
# ----------------------------------------------------

#
# == Read a register from the chip by address ==
#
	def _ReadRegister(self, hRegisterAddr):
	# -- Test for device instance before attempting use
		if ( self._oDeviceInst == None ):
		# -- Device not init'ed properly.  Return None
			return None

	# -- Test if IO is not locked -- needed for some reset conditions
		if ( self._bFlagLockIO == True ):
		# -- Loop wait wile the lock remains set
			_iLoopCounter = 0
			while ( self._bFlagLockIO == True ):
			# -- Wait one chip cycle
				time.sleep(self._fSleepMsec)
				_iLoopCounter += self._fSleepMsec
			# -- Check if the loop timeout was exceeded
				if ( ( self._bFlagLockIO == True ) and ( _iLoopCounter >= self._iTimeoutLockIOmsec ) ):
					if (self._bPrintDebug == True):	print "ReadRegister: I/O Lock timeout exceeded.  Read request aborted."
					return None

	# -- Shift in the register address three bits - see spec table 33
	#     bit0: not used		bits1-2: channel select
	#     bits3-6: register		bit7: not used
		_hShiftedRegisterAddr = (hRegisterAddr<<3)

	# -- Read the unsigned 8-bit value
		_hRegReadVal = self._oDeviceInst.readU8(_hShiftedRegisterAddr)

	# -- Print register value if debugging is enabled
		if (self._bPrintDebug == True):
			print inspect.stack(context=1)[1]
			print "Read register " + str(hex(hRegisterAddr)) + "(" + str(bin(hRegisterAddr)) + ") = " + str(hex(_hRegReadVal)) + "(" + str(bin(_hRegReadVal)) + ")"

	# -- Return the result...
		return _hRegReadVal



#
# == Write a value to a register on the chip by address ==
#
	def _WriteRegister(self, hRegisterAddr, hValue, bReadVerifyWrite = True):
	# -- Test for device instance before attempting use
		if ( self._oDeviceInst == None ):
		# -- Device not init'ed properly.  Return None
			return None

	# -- Test if IO is not locked -- needed for some reset conditions
		if ( self._bFlagLockIO == True ):
		# -- Loop wait wile the lock remains set
			_iLoopCounter = 0
			while ( self._bFlagLockIO == True ):
			# -- Wait one chip cycle
				time.sleep(self._fSleepMsec)
				_iLoopCounter += self._fSleepMsec
			# -- Check if the loop timeout was exceeded
				if ( ( self._bFlagLockIO == True ) and ( _iLoopCounter >= self._iTimeoutLockIOmsec ) ):
					if (self._bPrintDebug == True):	print "WriteRegister: I/O Lock timeout exceeded.  Write request aborted."
					return None

	# -- Shift in the register address three bits - see spec table 33
	#     bit0: not used		bits1-2: channel select
	#     bits3-6: register		bit7: not used
		_hShiftedRegisterAddr = (hRegisterAddr<<3)

	# -- Print register write value if debugging is enabled
		if (self._bPrintDebug == True):
			print inspect.stack(context=1)[1]
			print "Write register " + str(hex(hRegisterAddr)) + "(" + str(bin(hRegisterAddr)) + ") = " + str(hex(hValue)) + "(" + str(bin(hValue)) + ")"

	# -- Write out the unsigned 8-bit value and return the status
		try:
			if ( self._oDeviceInst.write8(_hShiftedRegisterAddr, hValue) == False ):	return False
		except:
			pass

	# -- Read the register to verify the write results, if enabled.
		if ( bReadVerifyWrite == True ):
			if ( self._ReadRegister(hRegisterAddr) != hValue ):
				if (self._bPrintDebug == True):	print "!! Register readback validation Failed !! -- Value returned does not match write."
				return False

	# -- If everything worked, return True
		return True




# ----------------------------------------------------
#   C L A S S   I N T E R N A L   F U N C T I O N S
# ----------------------------------------------------

#
# == LOCAL: Enable/Disable exposure of the Enhanced Register Set via LCR Register ==
#
	def _ExposeEnhancedRegisterSet(self, bExposeRegisterSet):
	# -- Expose the Register Set
		if ( (bExposeRegisterSet == True) and (self._hRegLCR == 0x00) ):
		# -- Read in the current LCR register
			_hRegLCR = self._ReadRegister(SC16IS750_REG_LCR)

		# -- Save the current LCR register state
			self._hRegLCR = _hRegLCR

		# -- Enable Enhanced Feature Register with LCR = 0xBF
			if ( self._WriteRegister(SC16IS750_REG_LCR, 0xbf) == False ):	return False

		elif ( (bExposeRegisterSet == False) and (self._hRegLCR != 0x00) ):
		# -- Retrieve the prior LCR register state
			_hRegLCR = self._hRegLCR
			self._hRegLCR = 0x00

		# -- Restore the LCR Register with to the previous state
			if ( self._WriteRegister(SC16IS750_REG_LCR, _hRegLCR) == False ):	return False

		else:
		# -- Something was in the wrong state...
			return False

	# -- If everything worked, return True
		return True



#
# == LOCAL: Enable/Disable the Enhanced Functions flag in the EFR register ==
#
	def _EnableEnhancedFunctionSet(self, bEnableAdvancedSet):
	# -- Enable Enhanced Register access
		if ( self._ExposeEnhancedRegisterSet(bExposeRegisterSet = True) == False ):	return False

	# -- Read the EFR register
		_hRegValue = self._ReadRegister(SC16IS750_REG_LCR_0XBF_EFR)

	# -- Enable/Disable Enhanced Function Set with EFR[4]
		if ( bEnableAdvancedSet == True ):
			_hRegValue |= 0x10
		else:
			_hRegValue &= 0xef

	# -- Write out the modified EFR register
		if ( self._WriteRegister(SC16IS750_REG_LCR_0XBF_EFR, _hRegValue) == False ):	return False

	# -- Disable Enhanced Register access
		if ( self._ExposeEnhancedRegisterSet(bExposeRegisterSet = False) == False ):	return False

	# -- If everything worked, return True
		return True



#
# == LOCAL: Enable special GPIO[4:7] pins for Modem flow control signals ==
#
	def _SetGPIO47forModemFlowcontrol(self, bModemUse):
	# -- Read the IOControl register
		_hRegValue = self._ReadRegister(SC16IS750_REG_IOCONTROL)

	# -- Set the GPIO[4:7] Modem Pins flag at IOControl[1]
		if ( bModemUse == True ):
			_hRegValue |= 0x02
		else:
			_hRegValue &= 0xfd

	# -- Write out the modified IOControl register
		if ( self._WriteRegister(SC16IS750_REG_IOCONTROL, _hRegValue) == False ):	return False

	# -- If everything worked, return True
		return True



#
# == LOCAL: Check that special GPIO[4:7] pins are set for Modem flow control signals ==
#
	def _CheckGPIO47forModemFlowcontrol(self):
	# -- Read the IOControl register
		_hRegIOC = self._ReadRegister(SC16IS750_REG_IOCONTROL)

	# -- Check the GPIO[4:7] Modem Pins flag at IOControl[1]
		if ( ( int(_hRegIOC) & (1 << 1) ) > 0 ):
			if (self._bPrintDebug == True):	print("_CheckGPIO47forModemFlowcontrol: IOControl[1] = 1; Pins are for modem hardware flow control.")
			return True
		else:
			if (self._bPrintDebug == True):	print("_CheckGPIO47forModemFlowcontrol: IOControl[1] = 0; Pins are in GPIO mode.")
			return False

	# -- If we fall through -- should never happen -- return false
		return False




# ----------------------------------------------------
#   C H I P   S E T U P
# ----------------------------------------------------

#
# == Issue a UART software reset to the chip ==
#
	def ResetDevice(self):
	# -- Read the IOControl register
		_hRegValue = self._ReadRegister(SC16IS750_REG_IOCONTROL)

	# -- Or in the software reset IOControl[3]
		_hRegValue |= 0x08

	# -- Write the reset bit - It will produce a write I/O error because the I2C bus
	#     receives a NAK.  So we can't verify the write was successful.
		self._WriteRegister(SC16IS750_REG_IOCONTROL, _hRegValue, bReadVerifyWrite = False)

	# -- Assume everything worked, return True
		return True



#
# == "Ping" the chip by a scratchpad (SPR) write and read test ==
#
	def Ping(self):
	# -- Write all bits high and verify read
		self._WriteRegister(SC16IS750_REG_SPR, 0xFF, bReadVerifyWrite = False)
		if (self._ReadRegister(SC16IS750_REG_SPR) != 0xFF):
			return False

	# -- Write 10101010 alternating bit pattern and verify read
		self._WriteRegister(SC16IS750_REG_SPR, 0xAA, bReadVerifyWrite = False)
		if (self._ReadRegister(SC16IS750_REG_SPR) != 0xAA):
			return False

	# -- Write 10000001 bookend bit pattern and verify read
		self._WriteRegister(SC16IS750_REG_SPR, 0x81, bReadVerifyWrite = False)
		if (self._ReadRegister(SC16IS750_REG_SPR) != 0x81):
			return False

	# -- We can read/write cleanly to scratch register, we can assume alive
		return True



#
# == Place the IC into Sleep Mode to greatly reduce power consumption. (See Spec. 7.6) ==
#
	def SetSleepState(self, bDiscardRxBuffer = False):
	# -- Check that there is no data is in the RX buffer
		if ( self.RxFifoBufferUsed() > 0 ):
			if (self._bPrintDebug == True):	print("SetSleepState: Data present in RX buffer; Cannot sleep now.")
			return False

	# -- Check that there is no data is in the TX buffers
		if ( GetLineStatus()['thr-tsr-empty'] == False ):
			if (self._bPrintDebug == True):	print("SetSleepState: Data present in TX hold or send buffers; Cannot sleep now.")
			return False

	# -- Enable enhanced function mode on EFR[4]
		if ( self._EnableEnhancedFunctionSet(bEnableAdvancedSet = True) == False ):	return False

	# -- Read in the current IER register
		_hRegIER = self._ReadRegister(SC16IS750_REG_IER)

	# -- Enable sleep mode with IER[4]
		_hRegIER |= 0x10

	# -- Write out the modified IER register
		if ( self._WriteRegister(SC16IS750_REG_IER, _hRegIER) == False ):	return False

	# -- If everything worked, return True
		return True



#
# == Check if the IC is in Sleep Mode from the IER[4] register state ==
#
	def GetSleepState(self):
	# -- Read in the current IER register
		_hRegIER = self._ReadRegister(SC16IS750_REG_IER)

	# -- Check if sleep mode bit is set on IER[4]
		if ( ( int(_hRegIER) & (1 << 4) ) > 0 ):
			if (self._bPrintDebug == True):	print("GetSleepState: UART sleeping.")
			return True
		else:
			if (self._bPrintDebug == True):	print("GetSleepState: UART awake.")
			return False



#
# == Wake the IC from Sleep Mode by de-asserting the IER[4] register state ==
#
	def SetWakeState(self):
	# -- Check if we are in a sleep state first
		if ( self.GetSleepState() == False ):
			if (self._bPrintDebug == True):	print("SetWakeState: UART not sleeping and already awake. Skipping wake protocol.")
			return True

	# -- Read in the current IER register
		_hRegIER = self._ReadRegister(SC16IS750_REG_IER)

	# -- Disable sleep mode with IER[4]
		_hRegIER &= 0xef

	# -- Write out the modified IER register
		if ( self._WriteRegister(SC16IS750_REG_IER, _hRegIER) == False ):	return False

	# -- If everything worked, return True
		return True




# ----------------------------------------------------
#   U A R T   S E T U P
# ----------------------------------------------------

#
# == Set the UART Wire Mode as RS-232 or Multidrop RS-485 (aka 9-bit mode) ==
#
	def SetMultidropMode(self, b9BitMode = False):
	# -- Read in the current EFCR register
		_hRegEFCR = self._ReadRegister(SC16IS750_REG_EFCR)

	# -- Modify the EFCR register as appropriate...
		if ( b9BitMode == True ):
		# -- Set the 9bit mode flag on EFCR[0]
			_hRegEFCR |= 0x01
			if (self._bPrintDebug == True):	print "SetMultidropMode: Setup in Multi-Drop RS-485 (aka 9-bit) mode."
		else:
		# -- Clear the 9bit mode flag on EFCR[0]
			_hRegEFCR &= 0xfe
			if (self._bPrintDebug == True):	print "SetMultidropMode: Setup in RS-232 mode."

	# -- Write out the modified EFCR register
		if ( self._WriteRegister(SC16IS750_REG_EFCR, _hRegEFCR) == False ):	return False

	# -- If everything worked, return True
		return True



#
# == Set the Baud rate for the UART ==
#
	def SetBaudrate(self, iBaud):
	# -- Check for sleep mode
		_bSleepState = self.GetSleepState()
		if ( _bSleepState == True ):
		# -- Wake the chip for setting the baud rate
			if ( SetWakeState() == False ):	return False

	# -- Read the LCR register
		_hRegLCR = self._ReadRegister(SC16IS750_REG_LCR)

	# -- Check that the LCR register is not in the special 0xBF state, else exit.
		if ( _hRegLCR == 0xbf ):
			return False

	# -- Find the clock divisor prescaler from register MCR[7]
		if ((self._ReadRegister(SC16IS750_REG_MCR) & 0x80) == 0):
		# -- logic 0 = divide-by-1 clock input
			_iClockDivisorPrescaler = 1
		else:
		# -- logic 1 = divide-by-4 clock input
			_iClockDivisorPrescaler = 4

	# -- Calculate the Clock Divisor - See specifications sec. 7.8 for detail
		_iClockDivisor = ( (SC16IS750_CRYSTAL_FREQ / _iClockDivisorPrescaler) / (iBaud * 16) )

	# -- Set LCR[7] to enable Divisor Latch and expose DLL & DLH registers
		_hRegLCR |= 0x80
		if ( self._WriteRegister(SC16IS750_REG_LCR, _hRegLCR) == False ):	return False

	# -- Write the first 8bits of the calculated clock divisor to the divisor latch LSB register
		if ( self._WriteRegister(SC16IS750_REG_LCR7_DLL, _iClockDivisor) == False ):	return False

	# -- Write the 8bit overflow of the calculated clock divisor to the divisor latch MSB register
		if ( self._WriteRegister(SC16IS750_REG_LCR7_DLH, (_iClockDivisor>>8)) == False ):	return False

	# -- Set LCR[7] to disable Divisor Latch again and return the remaining LCR register flags to their prior states
		_hRegLCR = self._ReadRegister(SC16IS750_REG_LCR)
		_hRegLCR &= 0x7F
		if ( self._WriteRegister(SC16IS750_REG_LCR,_hRegLCR) == False ):	return False

	# -- Calculate the real baud rate and the difference between desired and actual
		_iRealBaud = ( (SC16IS750_CRYSTAL_FREQ / _iClockDivisorPrescaler) / (16 * _iClockDivisor) )
		_eBaudError = (_iRealBaud - iBaud) * 1000 / iBaud

	# -- Print calculation debugging if enabled
		if (self._bPrintDebug == True):	print "Desired baudrate =" + str(iBaud) + " ; Calculated divisor =" + str(_iClockDivisor) + " ; Actual baudrate =" + str(_iRealBaud) + " ; Baudrate error =" + str(_eBaudError) + " ;"

	# -- If the chip was in sleep mode prior, return it to sleep state
		if ( _bSleepState == True ):
			if ( SetSleepState() == False ):	return False

	# -- If everything worked, return True
		return True



#
# == Set the line attributes for the UART ==
#
	def SetLine(self, iDataBits, sParityTyp, iStopBits):
	# -- Read the LCR register
		_hRegLCR = self._ReadRegister(SC16IS750_REG_LCR)

	# -- Check that the LCR register is not in the special 0xBF state, else exit.
		if ( _hRegLCR == 0xbf ):
			return False

	# -- Setup a Clear bitted var for the LCR
		_hRegLCR = 0x00;

	# -- Set data length on LCR[1] and LCR[0]  (See spec table 15)
		if   ( iDataBits == 8 ):
			_hRegLCR |= 0x03
		elif ( iDataBits == 7 ):
			_hRegLCR |= 0x02
		elif ( iDataBits == 6 ):
			_hRegLCR |= 0x01
		elif ( iDataBits == 5 ):
			_hRegLCR |= 0x00
		else:
			return False

	# -- Set the number of stop bits on LCR[2]  (See spec table 14)
		if   ( iStopBits == 2 ):
			_hRegLCR |= 0x04
		elif ( iStopBits == 1 ):
			_hRegLCR |= 0x00
		else:
			return False

	# -- Set parity method on LCR[5], LCR[4], and LCR[3]  (See spec table 13)
		sParityTyp = sParityTyp[:1]
		sParityTyp = sParityTyp.upper()
		if   ( sParityTyp == 'N' ):
			_hRegLCR |= 0x00
		elif ( sParityTyp == 'O' ):
			_hRegLCR |= 0x08
		elif ( sParityTyp == 'E' ):
			_hRegLCR |= 0x18
		else:
			return False

	# -- Write the line attributes into the LCR register
		if ( self._WriteRegister(SC16IS750_REG_LCR, _hRegLCR) == False ):	return False

	# -- If everything worked, return True
		return True



#
# == Enable/Configure/Disable FIFO Buffers ==
#
	def SetFifo(self, bFifoEnable = True, iRxFifoTriggerSpaces = 8, iTxFifoTriggerSpaces = 0):
	# -- Read in the current FCR register
		_hRegFCR = self._ReadRegister(SC16IS750_REG_FCR)

	# -- If both FIFO modes are False, we can just set the global FIFO flags to 0s
		if ( bFifoEnable == False ):
			if ( self._WriteRegister(SC16IS750_REG_FCR, 0x00, False) == True ):
				return True
			else:
				return False

	# -- If either FIFO mode is enabled, set the global FIFO flag on FCR[0]
		_hRegFCR |= 0x01

	# -- Set the receive FIFO buffer on FCR[6:7]
		if   (iRxFifoTriggerSpaces != 8):
			_hRegFCR |= 0x00
		elif (iRxFifoTriggerSpaces != 16):
			_hRegFCR |= 0x40
		elif (iRxFifoTriggerSpaces != 56):
			_hRegFCR |= 0x80
		elif (iRxFifoTriggerSpaces != 60):
			_hRegFCR |= 0xC0
		else:
			if (self._bPrintDebug == True):	print "Desired iRxFifoTriggerSpaces =" + str(iRxFifoTriggerSpaces) + " is not a valid input. Must be 8, 16, 56, or 60."
			return False

	# -- See if TxFifo trigger spaces was defined
		if ( iTxFifoTriggerSpaces > 0 ):
		# -- Enable enhanced function set as this is required for Tx FIFO
			if ( self._EnableEnhancedFunctionSet(bEnableAdvancedSet = True) == False ):	return False

		# -- Set the transmit FIFO buffer on FCR[4:5]
			if   (iTxFifoTriggerSpaces != 8):
				_hRegFCR |= 0x00
			elif (iTxFifoTriggerSpaces != 16):
				_hRegFCR |= 0x10
			elif (iTxFifoTriggerSpaces != 56):
				_hRegFCR |= 0x20
			elif (iTxFifoTriggerSpaces != 60):
				_hRegFCR |= 0x30
			else:
				if (self._bPrintDebug == True):	print "Desired iTxFifoTriggerSpaces =" + str(iTxFifoTriggerSpaces) + " is not a valid input. Must be 8, 16, 56, or 60."
				return False

	# -- Write out the modified FCR register
		if ( self._WriteRegister(SC16IS750_REG_FCR, _hRegFCR, False) == False ):	return False

	# -- If everything worked, return True
		return True



#
# == Enable the Automatic chip internal Hardware flow control with GPIO[4:7] control pins ==
#
	def SetAutoHardFlowcontrol(self):
		if (self._bPrintDebug == True):	print "SetAutoHardFlowcontrol: Enable Automatic Hardware Flow control."
	# -- Enable Enhanced Register access
		if ( self._ExposeEnhancedRegisterSet(bExposeRegisterSet = True) == False ):	return False

	# -- Read the EFR register
		_hRegValue = self._ReadRegister(SC16IS750_REG_LCR_0XBF_EFR)

	# -- Enable Auto RTS with EFR[6]
		_hRegValue |= 0x40

	# -- Enable Auto CTS with EFR[7]
		_hRegValue |= 0x80

	# -- Write out the modified EFR register
		if ( self._WriteRegister(SC16IS750_REG_LCR_0XBF_EFR, _hRegValue) == False ):	return False

	# -- Empty out the XON1 Register
		if ( self._WriteRegister(SC16IS750_REG_LCR_0XBF_XON1, 0x00) == False ):	return False

	# -- Empty out the XON2 Register
		if ( self._WriteRegister(SC16IS750_REG_LCR_0XBF_XON2, 0x00) == False ):	return False

	# -- Empty out the XOFF1 Register
		if ( self._WriteRegister(SC16IS750_REG_LCR_0XBF_XOFF1, 0x00) == False ):	return False

	# -- Empty out the XOFF2 Register
		if ( self._WriteRegister(SC16IS750_REG_LCR_0XBF_XOFF2, 0x00) == False ):	return False

	# -- Enable special GPIO[4:7] pins for Modem flow control signals
		if ( self._SetGPIO47forModemFlowcontrol(bModemUse = True) == False ):	return False

	# -- Disable Enhanced Register access
		if ( self._ExposeEnhancedRegisterSet(bExposeRegisterSet = False) == False ):	return False

	# -- If everything worked, return True
		return True



#
# == Enable the Hardware flow control GPIO[4:7] control pins ==
#
	def SetHardFlowcontrol(self):
		if (self._bPrintDebug == True):	print "SetHardFlowcontrol: Enable Hardware Flow control."
	# -- Enable Enhanced Register access
		if ( self._ExposeEnhancedRegisterSet(bExposeRegisterSet = True) == False ):	return False

	# -- Read the EFR register
		_hRegValue = self._ReadRegister(SC16IS750_REG_LCR_0XBF_EFR)

	# -- Disable Auto RTS with EFR[6]
		_hRegValue &= 0xbf

	# -- Disable Auto CTS with EFR[7]
		_hRegValue &= 0x7f

	# -- Write out the modified EFR register
		if ( self._WriteRegister(SC16IS750_REG_LCR_0XBF_EFR, _hRegValue) == False ):	return False

	# -- Empty out the XON1 Register
		if ( self._WriteRegister(SC16IS750_REG_LCR_0XBF_XON1, 0x00) == False ):	return False

	# -- Empty out the XON2 Register
		if ( self._WriteRegister(SC16IS750_REG_LCR_0XBF_XON2, 0x00) == False ):	return False

	# -- Empty out the XOFF1 Register
		if ( self._WriteRegister(SC16IS750_REG_LCR_0XBF_XOFF1, 0x00) == False ):	return False

	# -- Empty out the XOFF2 Register
		if ( self._WriteRegister(SC16IS750_REG_LCR_0XBF_XOFF2, 0x00) == False ):	return False

	# -- Enable special GPIO[4:7] pins for Modem flow control signals
		if ( self._SetGPIO47forModemFlowcontrol(bModemUse = True) == False ):	return False

	# -- Disable Enhanced Register access
		if ( self._ExposeEnhancedRegisterSet(bExposeRegisterSet = False) == False ):	return False

	# -- If everything worked, return True
		return True



#
# == Enable the Software flow control and define XOn/XOff fields ==
#
	def SetSoftFlowcontrol(self, bTxXOnOff, bRxXOnOff, hXOn1 = 0x11, hXOff1 = 0x13, hXOn2 = None, hXOff2 = None):
		if (self._bPrintDebug == True):	print "SetSoftFlowcontrol: Enable Software Flow control."
	# -- Make sure we have sane input before proceeding
		if ( (hXOn1 == None) and (hXOff1 == None) and (hXOn2 == None) and (hXOff2 == None) ):
			if (self._bPrintDebug == True):	print "SetSoftFlowcontrol: Invalid input.  No XOn/XOff chars defined."
			return False

	# -- Enable Enhanced Register access
		if ( self._ExposeEnhancedRegisterSet(bExposeRegisterSet = True) == False ):	return False

	# -- Read the EFR register
		_hRegEFR = self._ReadRegister(SC16IS750_REG_LCR_0XBF_EFR)

	# -- Disable Auto RTS with EFR[6]
		_hRegEFR &= 0xbf

	# -- Disable Auto CTS with EFR[7]
		_hRegEFR &= 0x7f

	# -- Reset EFR[0:3]
		_hRegEFR &= 0xF0

	# -- Enable software flow control method required with EFR[0:3]
		if   ( (bTxXOnOff == True) and (hXOn1 != None) and (hXOff1 != None) ):
			_hRegEFR |= 0x08
			if (self._bPrintDebug == True):	print "SetSoftFlowcontrol: TX On for XON/XOFF 1."
		if   ( (bTxXOnOff == True) and (hXOn2 != None) and (hXOff2 != None) ):
			_hRegEFR |= 0x04
			if (self._bPrintDebug == True):	print "SetSoftFlowcontrol: TX On for XON/XOFF 2."
		if   ( (bRxXOnOff == True) and (hXOn1 != None) and (hXOff1 != None) ):
			_hRegEFR |= 0x02
			if (self._bPrintDebug == True):	print "SetSoftFlowcontrol: RX On for XON/XOFF 1."
		if   ( (bRxXOnOff == True) and (hXOn2 != None) and (hXOff2 != None) ):
			_hRegEFR |= 0x01
			if (self._bPrintDebug == True):	print "SetSoftFlowcontrol: RX On for XON/XOFF 2."

	# -- Write out the modified EFR register
		if ( self._WriteRegister(SC16IS750_REG_LCR_0XBF_EFR, _hRegEFR) == False ):	return False

	# -- If defined, write out the XON1 Register
		if ( hXOn1 != None ):
			if ( self._WriteRegister(SC16IS750_REG_LCR_0XBF_XON1, hXOn1) == False ):	return False

	# -- If defined, write out the XON2 Register
		if ( hXOn2 != None ):
			if ( self._WriteRegister(SC16IS750_REG_LCR_0XBF_XON2, hXOn2) == False ):	return False

	# -- If defined, write out the XOFF1 Register
		if ( hXOff1 != None ):
			if ( self._WriteRegister(SC16IS750_REG_LCR_0XBF_XOFF1, hXOff1) == False ):	return False

	# -- If defined, write out the XOFF2 Register
		if ( hXOff2 != None ):
			if ( self._WriteRegister(SC16IS750_REG_LCR_0XBF_XOFF2, hXOff2) == False ):	return False

	# -- Disable special GPIO[4:7] pins for Modem flow control signals
		if ( self._SetGPIO47forModemFlowcontrol(bModemUse = False) == False ):	return False

	# -- Disable Enhanced Register access
		if ( self._ExposeEnhancedRegisterSet(bExposeRegisterSet = False) == False ):	return False

	# -- If everything worked, return True
		return True



#
# == Disable all flow control methods ==
#
	def SetNoFlowcontrol(self):
	# -- Enable Enhanced Register access
		if ( self._ExposeEnhancedRegisterSet(bExposeRegisterSet = True) == False ):	return False

	# -- Read the EFR register
		_hRegEFR = self._ReadRegister(SC16IS750_REG_LCR_0XBF_EFR)

	# -- Disable all bits in EFR except EFR[4:5]
		_hRegEFR &= 0x30

	# -- Write out the modified EFR register
		if ( self._WriteRegister(SC16IS750_REG_LCR_0XBF_EFR, _hRegEFR) == False ):	return False

	# -- Empty out the XON1 Register
		if ( self._WriteRegister(SC16IS750_REG_LCR_0XBF_XON1, 0x00) == False ):	return False

	# -- Empty out the XON2 Register
		if ( self._WriteRegister(SC16IS750_REG_LCR_0XBF_XON2, 0x00) == False ):	return False

	# -- Empty out the XOFF1 Register
		if ( self._WriteRegister(SC16IS750_REG_LCR_0XBF_XOFF1, 0x00) == False ):	return False

	# -- Empty out the XOFF2 Register
		if ( self._WriteRegister(SC16IS750_REG_LCR_0XBF_XOFF2, 0x00) == False ):	return False

	# -- Disable special GPIO[4:7] pins for Modem flow control signals
		if ( self._SetGPIO47forModemFlowcontrol(bModemUse = False) == False ):	return False

	# -- Disable Enhanced Register access
		if ( self._ExposeEnhancedRegisterSet(bExposeRegisterSet = False) == False ):	return False

	# -- If everything worked, return True
		return True




# ----------------------------------------------------
#   U A R T   O P E R A T I O N S   F U N C T I O N S
# ----------------------------------------------------

#
# == Clear and Reset the Transmit FIFO Buffer ==
#
	def ResetTxFifoBuffer(self):
	# -- Read the FCR register
		_hRegFCR = self._ReadRegister(SC16IS750_REG_FCR)

	# -- Check if reset is already in progress
		if ( ( int(_hRegFCR) & (1 << 2) ) > 0 ):
		# -- Flag still set, so abort.
			return False

	# -- Set the TX FIFO buffer clear flag on FCR[2]
		_hRegFCR |= 0x04

	# -- Write out the modified IOControl register
		if ( self._WriteRegister(SC16IS750_REG_FCR, _hRegFCR, False) == False ):	return False

	# -- FIFO reset requires at least two XTAL1 clock cycles 
		time.sleep(self._iTimeoutLockIOmsec)

	# -- If everything worked, return True
		return True



#
# == Clear and Reset the Receive FIFO Buffer ==
#
	def ResetRxFifoBuffer(self):
	# -- Read the FCR register
		_hRegFCR = self._ReadRegister(SC16IS750_REG_FCR)

	# -- Check if reset is already in progress
		if ( ( int(_hRegFCR) & (1 << 1) ) > 0 ):
		# -- Flag still set, so abort.
			return False

	# -- Set the RX FIFO buffer clear flag on FCR[1]
		_hRegFCR |= 0x02

	# -- Write out the modified IOControl register
		if ( self._WriteRegister(SC16IS750_REG_FCR, _hRegFCR, False) == False ):	return False

	# -- FIFO reset requires at least two XTAL1 clock cycles 
		time.sleep(self._iTimeoutLockIOmsec)

	# -- If everything worked, return True
		return True



#
# == Find how much data has been received into the FIFO Receive Buffer ==
#
	def RxFifoBufferUsed(self):
	# -- Read the RXLVL register
		_hFifoBufferBytes = self._ReadRegister(SC16IS750_REG_RXLVL)
		_iFifoBufferBytes = int(_hFifoBufferBytes)
		if (self._bPrintDebug == True):	print("RxFifoBufferUsed: " + str(_iFifoBufferBytes) + " bytes used.")

	# -- Return the value
		return _iFifoBufferBytes



#
# == Find how much space is available in the FIFO Transmit Buffer to write to ==
#
	def TxFifoBufferAvailable(self):
	# -- Read the TXLVL register
		_hFifoBufferBytes = self._ReadRegister(SC16IS750_REG_TXLVL)
		_iFifoBufferBytes = int(_hFifoBufferBytes)
		if (self._bPrintDebug == True):	print("TxFifoBufferAvailable: " + str(_iFifoBufferBytes) + " bytes available.")

	# -- Return the value
		return _iFifoBufferBytes



#
# == Read the Line Status flags ==
#
	def GetLineStatus(self):
	# -- Read in the current LSR register
		_hRegLSR = self._ReadRegister(SC16IS750_REG_LSR)

	# -- Create the LSR output dictionary
		_dLSR = {}

	# -- Loop each bit and add the state to the output dictionary
		for x in range(0, 8):
			if ( ( int(_hRegLSR) & (1 << x) ) > 0 ):
				_dLSR.update({ SC16IS750_REG_LSR_FIELDS[x]:True })
			else:
				_dLSR.update({ SC16IS750_REG_LSR_FIELDS[x]:False })

	# -- Return the output dictionary
		return _dLSR



#
# == Read the Modem Status flags ==
#
	def GetModemStatus(self):
	# -- Read the IOControl register
		_hRegIOC = self._ReadRegister(SC16IS750_REG_IOCONTROL)

	# -- Read in the current MSR register
		_hRegMSR = self._ReadRegister(SC16IS750_REG_MSR)

	# -- Create the MSR output dictionary
		_dMSR = {}

	# -- Define the MSR useful range based on Modem Pins (GPIO[4:7]) flag at IOControl[1]
		if ( ( int(_hRegIOC) & (1 << 1) ) > 0 ):
		# -- All MSR fields used
			_lMsrUsefulBitsRange = range(0, 8)
		else:
		# -- Only MSR[0:4] fields used. Other modem pins are I/O via IOState
			_lMsrUsefulBitsRange = range(0, 5)

	# -- Loop each bit in the defined range and add the state to the output dictionary
		for x in _lMsrUsefulBitsRange:
			if ( ( int(_hRegMSR) & (1 << x) ) > 0 ):
				_dMSR.update({ SC16IS750_REG_MSR_FIELDS[x]:True })
			else:
				_dMSR.update({ SC16IS750_REG_MSR_FIELDS[x]:False })

	# -- Return the output dictionary
		return _dMSR



#
# == Set/Release the line break state ==
#
	def SetLineBreak(self, bBreakConditionSet):
	# -- Read in the current LCR register
		_hRegLCR = self._ReadRegister(SC16IS750_REG_LCR)

	# -- Modify the LCR register as appropriate...
		if ( bBreakConditionSet == True ):
		# -- Set the break flag on LCR[6]
			_hRegLCR |= 0x40
		else:
		# -- Clear the break flag on LCR[6]
			_hRegLCR &= 0xbf

	# -- Write out the modified LCR register
		if ( self._WriteRegister(SC16IS750_REG_LCR, _hRegLCR) == False ):	return False

	# -- If everything worked, return True
		return True



#
# == Set the Modem RTS Status flag ==
#
	def SetModemRTS(self, bRtsLow):
	# -- Check that hardware flow control pins are enabled first
		if ( _CheckGPIO47forModemFlowcontrol == False ):
			return False

	# -- Read in the current MCR register
		_hRegMCR = self._ReadRegister(SC16IS750_REG_MCR)

	# -- Modify the MCR register as appropriate...
		if ( bRtsLow == True ):
		# -- Set RTS flag active (logic 1; LOW) on MCR[1]
			_hRegMCR |= 0x02
		else:
		# -- Set RTS flag inactive (logic 0; HIGH) on MCR[1]
			_hRegMCR &= 0xfd

	# -- Write out the modified MCR register
		if ( self._WriteRegister(SC16IS750_REG_MCR, _hRegMCR) == False ):	return False

	# -- If everything worked, return True
		return True



#
# == Set the Modem DTR Status flag ==
#
	def SetModemDTR(self, bDtrLow):
	# -- Check that hardware flow control pins are enabled first
		if ( _CheckGPIO47forModemFlowcontrol == False ):
			return False

	# -- Read in the current MCR register
		_hRegMCR = self._ReadRegister(SC16IS750_REG_MCR)

	# -- Modify the MCR register as appropriate...
		if ( bDtrLow == True ):
		# -- Set DTR flag active (logic 1; LOW) on MCR[0]
			_hRegMCR |= 0x01
		else:
		# -- Set DTR flag inactive (logic 0; HIGH) on MCR[0]
			_hRegMCR &= 0xfe

	# -- Write out the modified MCR register
		if ( self._WriteRegister(SC16IS750_REG_MCR, _hRegMCR) == False ):	return False

	# -- If everything worked, return True
		return True




# ----------------------------------------------------
#   U A R T   F R O N T - E N D   F U N C T I O N S
# ----------------------------------------------------

#
# == Connect Wrapper Function to simplify use... ==
#
	def Connect(self, iBaudRate = 9600, eParity = 'N', iDataBits = 8, iStopBits = 1, eFlowControl = 'NONE', bRS485Mode = False):

		if (self._bPrintDebug == True):	print("Connect: INIT iBaudRate = " + str(iBaudRate) + " ; eParity = " + str(eParity) + " ; iDataBits = " + str(iDataBits) + " ; iStopBits = " + str(iStopBits) + " ; eFlowControl = " + str(eFlowControl) + " ; bRS485Mode = " + str(bRS485Mode) + " ;")

		if (self._bPrintDebug == True):	print("Connect: Set RS-232 / RS-485 Multidrop mode.")
		if ( self.SetMultidropMode(bRS485Mode) == False ):
			if (self._bPrintDebug == True):	print("Connect: Set RS-232 / RS-485 Multidrop mode failed.")
			return False

		if (self._bPrintDebug == True):	print("Connect: Set Baud Rate.")
		if ( self.SetBaudrate(iBaudRate) == False ):
			if (self._bPrintDebug == True):	print("Connect: Set Baud Rate failed.")
			return False

		if (self._bPrintDebug == True):	print("Connect: Set Line.")
		if ( self.SetLine(iDataBits, eParity, iStopBits) == False ):
			if (self._bPrintDebug == True):	print("Connect: Set Line failed.")
			return False

		if (self._bPrintDebug == True):	print("Connect: Set FIFO.")
		if ( self.SetFifo(True) == False ):
			if (self._bPrintDebug == True):	print("Connect: Set FIFO failed.")
			return False

		eFlowControl = eFlowControl[:4]
		eFlowControl = eFlowControl.upper()
		if   ( eFlowControl == 'NONE' ):
			if (self._bPrintDebug == True):	print("Connect: Set No Flow Control.")
			if ( self.SetNoFlowcontrol() == False ):
				if (self._bPrintDebug == True):	print("Connect: Set No Flow Control failed.")
				return False
		elif ( eFlowControl == 'SOFT' ):
			if (self._bPrintDebug == True):	print("Connect: Set XOn/XOff Flow Control.")
			if ( self.SetSoftFlowcontrol(bTxXOnOff = True, bRxXOnOff = True) == False ):
				if (self._bPrintDebug == True):	print("Connect: Set XOn/XOff Flow Control failed.")
				return False
		elif ( eFlowControl == 'HARD' ):
			if (self._bPrintDebug == True):	print("Connect: Set Hardware Flow Control.")
			if ( self.SetHardFlowcontrol() == False ):
				if (self._bPrintDebug == True):	print("Connect: Set Hardware Flow Control failed.")
				return False
		elif ( eFlowControl == 'AUTO' ):
			if (self._bPrintDebug == True):	print("Connect: Set Automatic Chip Internal Hardware Flow Control.")
			if ( self.SetAutoHardFlowcontrol() == False ):
				if (self._bPrintDebug == True):	print("Connect: Set Automatic Chip Internal Hardware Flow Control failed.")
				return False
		else:
			if (self._bPrintDebug == True):	print("Connect: Invalid flow control setting")
			return False

	# -- If everything worked, return True
		return True



#
# == Write a Hex defined Byte to the UART ==
#
	def WriteByte(self, hValue, bDieOnNoTxBufferSpace = False):
	# -- Check if there is transmit hold buffer space available to write to
		if ( self.TxFifoBufferAvailable() == 0 ):
		# -- If requested, fail out if there is no space to write
			if ( bDieOnNoTxBufferSpace == True ):
				if (self._bPrintDebug == True):	print("WriteByte: No available space in transmit hold buffer. Aborting on request.")
				return False
			else:
		# -- Else, wait and loop each clock cycle until the transmit hold buffer is empty
				if (self._bPrintDebug == True):	print("WriteByte: No available space in transmit hold buffer. Waiting for THR empty flag.")
				while ( GetLineStatus()['thr-empty'] == False ):
				# -- Wait one chip cycle
					time.sleep(self._fSleepMsec)

	# -- Write the data byte to the THR Register
		if ( self._WriteRegister(SC16IS750_REG_THR, hValue, bReadVerifyWrite = False) == False ):	return False
		if (self._bPrintDebug == True):	print("++++++++++++Data sent")

	# -- If everything worked, return True
		return True



#
# == Read a Hex defined Byte to the UART ==
#
	def ReadByte(self, bDieOnNoRxBufferData = True):
	# -- Check if there is data in the receive buffer to read
		if ( self.RxFifoBufferUsed() == 0 ):
		# -- If requested, fail out if there is nothing to read
			if ( bDieOnNoRxBufferData == True ):
				if (self._bPrintDebug == True):	print("ReadByte: No data available in buffer to read. Aborting on request.")
				return None
			else:
		# -- Else, wait and loop each clock cycle until the receive buffer is not empty
				if (self._bPrintDebug == True):	print("ReadByte: No data available in buffer to read. Waiting for RXLVL > 0.")
				while ( self.RxFifoBufferUsed() == 0 ):
				# -- Wait one chip cycle
					time.sleep(self._fSleepMsec)

	# -- Read the data byte from the RHR Register
		if (self._bPrintDebug == True):	print("***********Data available***********")
		hValue = self._ReadRegister(SC16IS750_REG_RHR)

	# -- If everything worked, return the value
		return hValue




###################### ---------------------------------




# ----------------------------------------------------
#   G P I O   S E T U P   F U N C T I O N S
# ----------------------------------------------------


# ----------------------------------------------------
#   G P I O   O P E R A T I O N S   F U N C T I O N S
# ----------------------------------------------------

