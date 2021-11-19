# SPDX-License-Identifier: BSD-3-Clause

"""Silicon Labs C2 support for PyFdti"""

from typing import Union
from .ftdi import Ftdi


class C2Error(Exception):
    """Generic C2 error."""
    pass


class C2Controller:
    """C2 master of an FTDI device"""

    CK_BIT = 0x01  #AD0 - C2CK
    D_O_BIT = 0x02  #AD1 - C2D output
    D_I_BIT = 0x04  #AD2 - C2D input
    D_FB_BIT = 0x20  #AD5 - C2D input for end of WAIT detection using CLK_WAIT_ON_HIGH MPSSE op
    FTDI_PIPE_LEN = 512
    # C2 instructions
    INS_READ_DATA = 0
    INS_WRITE_DATA = 1
    INS_READ_ADDR = 2
    INS_WRITE_ADDR  =3

    # Private API
    def __init__(self, frequency: float = 20.0E6):  
        # At 20MHz the shortest C2CK pulse is 30ns 
        # C2 specification states 20ns minimum
        self._ftdi = Ftdi()
        self._frequency = frequency
        self._write_buff = bytearray()

    # Public API
    def configure(self, url: str) -> None:
        """Configure the FTDI interface as a C2 controller"""
        self._ftdi.open_mpsse_from_url(
            url, direction=0, frequency=self._frequency)
        # FTDI requires to initialize all GPIOs before MPSSE kicks in
        cmd = bytearray((Ftdi.SET_BITS_LOW, self.CK_BIT | self.D_O_BIT, 0))
        self._ftdi.write_data(cmd)
        self._ftdi.enable_3phase_clock(True)

    def close(self, freeze: bool = False) -> None:
        """Close the JTAG interface/port.

           :param freeze: if set, FTDI port is not reset to its default
                          state on close. This means the port is left with
                          its current configuration and output signals.
                          This feature should not be used except for very
                          specific needs.
        """
        if self._ftdi.is_connected:
            self._ftdi.close(freeze)

    def purge(self) -> None:
        self._ftdi.purge_buffers()

    @property
    def ftdi(self) -> Ftdi:
        """Return the Ftdi instance.

           :return: the Ftdi instance
        """
        return self._ftdi

    def _stack_cmd(self, cmd: Union[bytes, bytearray]):
        if not isinstance(cmd, (bytes, bytearray)):
            raise TypeError('Expect bytes or bytearray')
        if not self._ftdi:
            raise C2Error("FTDI controller terminated")
        # Currrent buffer + new command + send_immediate
        if (len(self._write_buff)+len(cmd)+1) >= C2Controller.FTDI_PIPE_LEN:
            self.sync()
        self._write_buff.extend(cmd)

    def sync(self) -> None:
        if not self._ftdi.is_connected:
            raise C2Error("FTDI controller terminated")
        if self._write_buff:
            self._ftdi.write_data(self._write_buff)
            self._write_buff = bytearray()

    def claim(self):
        """Activate C2 bus"""
        # TODO: implement C2 pin sharing
        cmd = bytearray((Ftdi.SET_BITS_LOW,
                self.CK_BIT | self.D_O_BIT,
                self.CK_BIT))
        self._ftdi.write_data(cmd)

    def release(self):
        """Deactivate C2 bus"""
        # TODO: implement C2 pin sharing
        self.sync()
        cmd = bytearray((Ftdi.SET_BITS_LOW,
                self.CK_BIT | self.D_O_BIT,
                0))
        self._ftdi.write_data(cmd)

    def reset(self) -> None:
        """Reset the Target"""
        if not self._ftdi.is_connected:
            raise C2Error("FTDI controller terminated")
        self.sync()
        # long pulse nRST (C2CK)
        # C2 spec requires >20us pulse width and >2us pause after the pulse
        # USB 2.0 schedule period is 125us - well within limits, no delays needed
        cmd = bytearray((Ftdi.SET_BITS_LOW, 0 | self.D_O_BIT, self.CK_BIT))
        self._ftdi.write_data(cmd)
        cmd = bytearray((Ftdi.SET_BITS_LOW, self.CK_BIT | self.D_O_BIT, self.CK_BIT))
        self._ftdi.write_data(cmd)

    def write_addr(self, addr: int):
        """Queued C2 Write Address command.
    
            :param addr: C2 register address to write
            The bus must be claimed already
        """
        cmd = bytearray((
                Ftdi.CLK_BITS_NO_DATA, 0,                           # START
                Ftdi.SET_BITS_LOW, self.CK_BIT | self.D_O_BIT, self.CK_BIT | self.D_O_BIT, # drive C2D
                Ftdi.WRITE_BITS_PVE_LSB, 1, self.INS_WRITE_ADDR,    # INS
                Ftdi.WRITE_BYTES_PVE_LSB, 0, 0, addr,               # ADDRESS
                Ftdi.SET_BITS_LOW, self.CK_BIT | self.D_O_BIT, self.CK_BIT, # release C2D
                Ftdi.CLK_BITS_NO_DATA, 0                            # STOP
                ))
        self._stack_cmd(cmd)

    def read_addr(self):
        """Immediate C2 Read Address command.

            :return: address reg value
            The bus must be claimed already
        """
        # TODO: make async
        cmd = bytearray((
                Ftdi.CLK_BITS_NO_DATA, 0,                           # START
                Ftdi.SET_BITS_LOW, self.CK_BIT | self.D_O_BIT, self.CK_BIT | self.D_O_BIT, # drive C2D
                Ftdi.WRITE_BITS_PVE_LSB, 1, self.INS_READ_ADDR,     # INS
                Ftdi.SET_BITS_LOW, self.CK_BIT | self.D_O_BIT, self.CK_BIT, # release C2D
                Ftdi.READ_BYTES_PVE_LSB, 0, 0,                      # ADDRESS
                Ftdi.CLK_BITS_NO_DATA, 0                            # STOP
                ))
        self._stack_cmd(cmd)
        self.sync()
        data = self._ftdi.read_data_bytes(1, 4)
        return data[0]

    def write_data_byte(self, val: int):
        """Queued C2 Write Data single byte command.
    
            :param val: data byte to write
            The bus must be claimed already
        """
        cmd = bytearray((
                Ftdi.CLK_BITS_NO_DATA, 0,                           # START
                Ftdi.SET_BITS_LOW, self.CK_BIT | self.D_O_BIT, self.CK_BIT | self.D_O_BIT, # drive C2D
                Ftdi.WRITE_BITS_PVE_LSB, 3, self.INS_WRITE_DATA,    # INS+LENGTH0
                Ftdi.WRITE_BYTES_PVE_LSB, 0, 0, val,                # DATA
                Ftdi.SET_BITS_LOW, self.CK_BIT | self.D_O_BIT, self.CK_BIT, # release C2D
                Ftdi.CLK_WAIT_ON_HIGH,                              # WAIT
                Ftdi.CLK_BITS_NO_DATA, 0                            # STOP
                ))
        self._stack_cmd(cmd)

    def read_data_byte(self):
        """Immediate C2 Read Data signe byte command.

            :return: value
            The bus must be claimed already
        """
        # TODO: make async
        cmd = bytearray((
                Ftdi.CLK_BITS_NO_DATA, 0,                           # START
                Ftdi.SET_BITS_LOW, self.CK_BIT | self.D_O_BIT, self.CK_BIT | self.D_O_BIT, # drive C2D
                Ftdi.WRITE_BITS_PVE_LSB, 3, self.INS_READ_DATA,     # INS+LENGTH0
                Ftdi.SET_BITS_LOW, self.CK_BIT | self.D_O_BIT, self.CK_BIT, # release C2D
                Ftdi.CLK_WAIT_ON_HIGH,                              # WAIT
                Ftdi.READ_BYTES_PVE_LSB, 0, 0,                      # DATA
                Ftdi.CLK_BITS_NO_DATA, 0                            # STOP
                ))
        self._stack_cmd(cmd)
        self.sync()
        data = self._ftdi.read_data_bytes(1, 4)
        return data[0]
