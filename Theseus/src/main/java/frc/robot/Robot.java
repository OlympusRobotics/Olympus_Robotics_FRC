// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⣀⣀⣤⣤⣤⣶⣶⣶⣶⠶⢠⣤⣤⣤⣤⣤⣄⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢠⣴⣾⠛⠛⠋⠉⠉⠉⠀⠀⠀⠀⠀⠈⠉⠉⠉⠉⠉⠛⣿⣶⡄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣰⡿⠛⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠙⢿⣦⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⣾⠟⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣤⣤⣀⠀⠀⠀⠀⠙⠻⣷⣄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⣰⡿⠁⠀⠀⠀⠀⠀⠀⠀⢀⣤⡀⠀⠀⠀⠀⣀⣀⣴⣿⠛⠉⠻⢿⣦⡀⠀⠀⠀⠘⣿⡆⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⣰⡿⠁⠀⣶⡶⠿⢶⢶⣶⣶⣾⠟⠋⠀⠀⠀⠘⠛⠛⠋⠀⠀⠀⠀⠈⠿⠃⠀⠀⠀⠀⠹⣿⣄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⢀⣿⠇⠀⢀⠀⠀⢀⣀⡀⡀⠀⠀⠤⠀⠀⠀⠀⠀⢨⠀⠀⠀⠀⠀⠀⠀⠀⠤⠀⠀⠀⠀⠀  ⠸⣿⡄⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⢸⡿⠀⠀⣿⠿⠿⠟⠛⣿⣿⣿⣿⡿⠂⠀⠀⠀⠀⠺⠿⠿⣿⣿⣿⣿⡿⠿⢿⣾⠀⠀  ⠀⣿⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⢸⡇⠠⠀⠀⠀⠀⠀⠀⠙⠿⠿⠋⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠻⠿⠿⠁⠀⠀⠀⠀⠀⠀⠀   ⠸⣿⡄⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⣼⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀     ⢀⣿⠇⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⢿⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠠⣾⣿⣿⡿⢿⣦⠀⠀    ⢸⣿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠸⣷⡄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⣿⠇⠀⠁⠀    ⢀⣿⠇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⢻⣷⡀⠀⠀⠀⢀⣄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀ ⠀⠀⣀⣠⣴⡾⠟⠁⠀⠀ ⠀ ⢀⣾⡟⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⢿⣷⡀⠀⠀⠈⢿⣿⣶⣶⣶⣶⣆⣀⣀⣰⣶⣶⣶⣾⣿⢿⠿⠁⠀⠀⠀⠀⠀⢀⣾⡿⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠻⣷⣄⠀⠀⠀⠀⠈⠈⠉⠉⢩⣭⣉⣉⣉⣉⣠⣤⣶⡾⠃⠀⠀⠀⠀⠀⣲⡿⠛⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠻⣷⣄⣀⠀⠀⠀⠀⠀⠀⠉⠙⠛⠋⠉⠉⠁⠀⠀⠀⠀⠀⣀⣴⣾⣟⣁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣈⣿⠿⠿⢶⣶⣤⣤⣤⣄⣀⣀⣠⣤⣤⣤⣴⣶⡶⠿⠿⠛⠋⠈⠛⠻⣷⣄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣾⡟⠙⠀⠀⠀⠀⠈⠉⠉⠉⠋⠙⠉⠉⠉⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠻⣷⡤⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⣿⠏⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠙⣿⣆⡀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⢀⣼⡟⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⣦⣀⠀⢰⣄⣤⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠘⢿⣦⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⣰⡿⠋⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⣾⠛⠉⢻⣶⣿⠋⠛⢿⣦⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⢻⣧⡀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⢀⣼⡿⠉⠀⠀⠀⣀⠀⠀⠀⠀⠀⠀⣠⣾⣿⣤⡶⠿⠛⠛⠛⠻⣦⡈⣿⣷⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢻⣷⣀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⢠⣾⠟⠀⠀⠀⠀⢰⣿⡆⠀⠀⣠⣶⠾⡟⢻⣩⣼⣧⣦⣤⣤⣀⣀⣀⣿⣿⣿⣿⣦⡄⠀⠀⠀⣀⣦⠀⠀⠀⠀⠀⢻⣷⡀⠀⠀⠀⠀
⠀⠀⠀⢰⣿⠃⠀⠀⠀⠀⠀⠀⣿⡇⠀⣴⡿⠁⢈⣴⠟⠉⠀⢀⣀⣤⣭⣿⣭⣍⡙⠀⠀⠉⠻⣿⣴⡀⠀⣿⣿⠀⠀⠀⠀⠀⠀⠻⣷⡀⠀⠀⠀
⠀⠀⢠⣿⠇⠀⠀⠀⠀⠀⠀⠠⣿⣅⣼⠟⠀⠀⠺⣿⣤⣴⠾⠟⠋⠉⠀⠀⠉⠙⠻⢷⣦⡀⠀⠸⣿⣧⠀⢸⣿⠀⠀⠀⠀⠀⠀⠐⢻⣿⡀⠀⠀
⠀⠀⣼⡿⢀⠀⠀⠀⠀⠀⠀⢰⣿⣿⠟⠀⠀⠀⠀⠻⠃⠀⠀⣀⣀⣶⣤⣄⣀⠀⠀⣀⣿⡇⠀⠀⢸⣿⡆⢸⣿⠀⠀⠀⠀⠀⠀⠀⠀⢿⣧⠀⠀
⠀⣼⡿⠁⠀⠀⠀⠀⠀⠀⢀⣼⡿⠁⠀⠀⠀⠀⠀⣠⣶⠾⠿⠛⠛⠉⠉⠙⠛⠿⠟⠛⠋⠀⠀⠀⠀⠙⣿⣼⣿⡇⠀⠀⠀⠀⠀⠀⠀⢸⣿⡇⠀
⢠⣿⠁⠀⠀⢠⣷⣶⣶⡾⠿⠛⠁⠀⠀⠀⠀⠀⠀⣿⣧⣄⡀⢀⣠⣤⣤⣤⣤⣀⡀⠀⠀⠀⠀⠀⠀⠀⠈⢿⣿⣧⣶⣄⣦⡀⠀⠀⠀⠘⣿⡏⠀
⣾⡏⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠉⠙⠛⠛⠋⠉⠀⠈⠉⠙⢿⣦⣀⠀⠀⠀⠀⠀⠀⠀⠉⠙⠛⠙⠉⠀⠀⠀⠀⠀⢻⣷⠀
⣿⡅⠄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⣴⡿⠿⠿⠿⠷⣶⣦⣤⣀⣀⣹⣯⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⣿⠄
⢿⣿⣶⣀⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢿⣇⣀⡀⠀⢀⣀⣀⣀⣩⣍⣛⠋⡁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣸⣿⠀
*/

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;
    private final Field2d field;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        Logger.recordMetadata("ProjectName", "Theseus");
        Logger.recordMetadata("TeamNumber", "4982");

        if (isReal()) {
            Logger.addDataReceiver(new WPILOGWriter());
            Logger.addDataReceiver(new NT4Publisher());
        } else {
            Logger.addDataReceiver(new NT4Publisher());
        }

        Logger.start();

        m_robotContainer = new RobotContainer();
        field = new Field2d();
    }

    @Override
    public void robotInit() {
        
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();
        SmartDashboard.putData("Field", field);
        field.setRobotPose(m_robotContainer.drivetrain.getState().Pose);
    }

    @Override
    public void disabledInit() {
        //m_robotContainer.intake.m_inkMot.setPosition(0);
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        /* m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        } */
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }

        m_robotContainer.intake.m_inkMot.setPosition(0);
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {

    }

    @Override
    public void testExit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
