package frc.robot.hardware;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PowerDistributionPanel;

/**
 * Extended hardware map for advanced / debugging control
 * Not necessary to import unless advanced control needed
 */
public class HardwareMapEx {
    public static final PowerDistributionPanel kPowerDistributionPanel = new PowerDistributionPanel();
    public static final Compressor kCompressor = new Compressor(HardwareIDs.kCompressor.value);
}