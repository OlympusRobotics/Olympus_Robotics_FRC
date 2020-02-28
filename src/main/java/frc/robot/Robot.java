/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.controls.DriveControlBase;
import frc.robot.hardware.HardwareMap;
import frc.robot.hardware.HardwareMapEx;

import java.util.HashMap;
import java.util.Map;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends IterativeRobot {
    /**
     * private static final String kDefaultAuto = "Default"; private static final
     * String kCustomAuto = "My Auto"; private String m_autoSelected; private final
     * SendableChooser<String> m_chooser = new SendableChooser<>();
     **/
    /*private static TankControls s_tankControls = new TankControls(HardwareMap.kDrive, HardwareMap.kRightJoystick,
            HardwareMap.kRightJoystick);
    private static ArcadeControls s_arcadeControls = new ArcadeControls(HardwareMap.kDrive, HardwareMap.kRightJoystick);
    private static CurvatureControls s_curvatureControls = new CurvatureControls(HardwareMap.kDrive,
            HardwareMap.kRightJoystick);*/
    private static DriveControlBase s_driveBase;
    private static Map<Class<? extends DriveControlBase>, DriveControlBase> s_driveClassMap = new HashMap<Class<? extends DriveControlBase>, DriveControlBase>();
    private static volatile SendableChooser<Boolean> s_debugEnabled = new SendableChooser<Boolean>();
    private static long m_lastTime = System.nanoTime();

    static {
        //s_driveClassMap.put(TankControls.class, s_tankControls);
        //s_driveClassMap.put(ArcadeControls.class, s_arcadeControls);
        //s_driveClassMap.put(CurvatureControls.class, s_curvatureControls);
        //s_driveBase = s_driveClassMap.get(DriveControlBase.getDefaultControlSystem());
        s_debugEnabled.setDefaultOption("False", false);
        s_debugEnabled.addOption("True", true);
    }

    /**
     * Puts advanced debug information to smart dashboard
     */
    private static final void putDebug() {
        //SmartDashboard.putNumber("Compressor current", HardwareMapEx.kCompressor.getCompressorCurrent());
        SmartDashboard.putNumber("PDP temperature", HardwareMapEx.kPowerDistributionPanel.getTemperature());
        SmartDashboard.putNumber("PDP current", HardwareMapEx.kPowerDistributionPanel.getTotalCurrent());
        SmartDashboard.putNumber("PDP energy", HardwareMapEx.kPowerDistributionPanel.getTotalEnergy());
        SmartDashboard.putNumber("PDP power", HardwareMapEx.kPowerDistributionPanel.getTotalPower());
        SmartDashboard.putNumber("PDP voltage", HardwareMapEx.kPowerDistributionPanel.getVoltage());
        for (int i = 0; i <= 15; i++) {
            double channelCurrent = HardwareMapEx.kPowerDistributionPanel.getCurrent(i);
            if (channelCurrent != 0) {
                SmartDashboard.putNumber("PDP current (channel " + i + ")", channelCurrent);
            }
        }
    }

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        SmartDashboard.putData(s_debugEnabled);
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like diagnostics that you want ran during disabled, autonomous,
     * teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and SmartDashboard integrated updating.
     */

    public void mainPeriodic() {
        //SmartDashboard.putString("Current control system", s_driveBase.getName());
        //s_driveBase.update();
        //Class<? extends DriveControlBase> switchControlSystem = s_driveBase.changeRequested();
        //if (switchControlSystem != null) {
        //    s_driveBase = s_driveClassMap.get(switchControlSystem);
        //}
        if (HardwareMap.kXbox.getAButton()) {
            HardwareMap.kIntakeMotor.set(0.3);
        }
        else if (HardwareMap.kXbox.getBButton()) {
            HardwareMap.kIntakeMotor.set(-0.3);
        }
        else {
            HardwareMap.kIntakeMotor.set(0.0);
        }

        if (HardwareMap.kXbox.getXButton()) {
            HardwareMap.kBeltMotor.set(1.0);
        }
        else if (HardwareMap.kXbox.getYButton()) {
            HardwareMap.kBeltMotor.set(-1.0);
        }
        else {
            HardwareMap.kBeltMotor.set(0.0);
        }

        if (Math.abs(HardwareMap.kXbox.getY(GenericHID.Hand.kLeft)) > 0.1) {
            HardwareMap.kOutputMotor1.set(HardwareMap.kXbox.getY(GenericHID.Hand.kLeft));
            HardwareMap.kOutputMotor2.set(HardwareMap.kXbox.getY(GenericHID.Hand.kLeft));
        }
        else {
            HardwareMap.kOutputMotor1.set(0.0);
            HardwareMap.kOutputMotor2.set(0.0);
        }
        if (System.nanoTime() - m_lastTime > 1e7) {
            m_lastTime = System.nanoTime();
        }
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable chooser
     * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
     * remove all of the chooser code and uncomment the getString line to get the
     * auto name from the text box below the Gyro
     *
     * <p>
     * You can add additional auto modes by adding additional comparisons to the
     * switch structure below with additional strings. If using the SendableChooser
     * make sure to add them to the chooser code above as well.
     */
    @Override
    public void autonomousInit() {
        // TODO
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        mainPeriodic();
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        mainPeriodic();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
        mainPeriodic();
        putDebug();
    }
}
