import wpilib
import commands2

class irTest(commands2.Subsystem):
    def __init__(self):
        self.ir = wpilib.DigitalInput(0)


        super().__init__()

    def test(self):
        return self.ir.get()