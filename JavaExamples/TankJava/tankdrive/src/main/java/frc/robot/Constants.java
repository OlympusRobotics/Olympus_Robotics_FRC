/* Place to add robot constants specific to the robot being programmed. */
package frc.robot;

import edu.wpi.first.math.util.Units;


public class Constants {
    //Controller ID
    public static final int DriverController = 0;
    public static final int OperatorController = 1;

    //Device CAN IDs
    public static final int leftMotor1_ID = 0;
    public static final int rightMotor1_ID = 1;
    public static final int leftMotor2_ID = 2;
    public static final int rightMotor2_ID = 3;

    //Current Limits
    public static final int driveCurrentLimit = 40;

    //Robot Track Width
    public static final double trackWidth = Units.inchesToMeters(27.0);

}
