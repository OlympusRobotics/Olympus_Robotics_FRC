import ntcore
import commands2

class limelight(commands2.Subsystem):
    def __init__(self):
        #Call the limelight network tables instance.
        self.table = ntcore.NetworkTableInstance.getDefault().getTable("limelight")
        super().__init__()

    def getHW(self):
        """ 
        Get hardware telemetry values (fps, cpu temp, ram usage, temp)
        """
        return self.table.getEntry("hw").getDoubleArray([0,0,0,0])
        
    def isTooHot(self):
        hw = self.getHW()
        cpuTemp = hw[1]
        deviceTemp = hw[3]
        
        if (cpuTemp > 70 or deviceTemp > 70):
            return True
        else:
            return False

    def aim(self):
        """  
        Auto aim method (Tracks both apriltags, Coral, and Algae)
        """
        tx = self.table.getEntry("tx").getDouble(0)
        kP = 0.15

        angularVel = tx * kP
        return angularVel
    
    def aiPipeline(self):
        """ 
        Sets the pipeline meant for ai tracking
        """
        return self.table.getEntry("pipeline").setInteger(1)
    
    def aprilTagPipeline(self):
        """ 
        Sets the pipeline meant for apriltag tracking
        """
        return self.table.getEntry("pipeline").setInteger(0)
    
    def targetCheck(self):
        """ 
        Checks to see if the limelight detects a target (apriltag or algae).
        """
        if (self.table.getEntry("tx") != 0):
            return True
        else:
            return False