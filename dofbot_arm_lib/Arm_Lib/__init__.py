from .arm_driver import Arm_Driver

# Vendor nodes do `from Arm_Lib import Arm_Device` against Yahboom's I2C
# expansion-board class. Arm_Driver implements that same API over the serial
# bus-servo protocol, so the alias lets them import unchanged.
Arm_Device = Arm_Driver

__all__ = ['Arm_Driver', 'Arm_Device']
