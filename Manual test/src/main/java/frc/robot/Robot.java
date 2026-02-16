// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/*
 ⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⣀⣀⠤⠤⠒⠒⠒⠒⠲⠦⣀⣀⡀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⢀⡠⠐⠊⠉⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠙⢦⡀⠀⠀⠀⠀
⠀⢀⡶⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢳⠀⠀
⠀⡏⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢹⡄
⢸⠁⡤⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢻
⡏⢠⠁⠱⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸
⡇⡞⠀⠀⢣⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠠⠤⢀⠀⠀⠀⠀⠀⢀⡠⠀⠘⢆⢻
⡗⡇⠀⠀⠈⢆⠀⠀⠀⠀⠀⠀⢀⣀⡠⠖⠒⠒⠢⣄⠁⠀⢀⢀⣠⠞⠉⠑⠢⣜⠀
⢠⠃⠀⠀⠀⠈⣆⠀⠀⠀⠀⢠⣿⡏⠀⠀⠀⢀⣀⠈⠆⠐⠁⠈⡏⠀⠀⢀⣤⡜⡆
⢸⠀⠀⠀⠀⠀⠀⠀⠀⠀⠒⣿⣿⡆⠀⠀⠀⣛⣿⡇⣤⠀⠀⠀⠑⡀⠀⠘⣘⣃⠃
⠀⢇⠀⠀⡀⠀⠀⠀⠀⠀⠀⠸⣇⠙⢦⣀⠀⠈⣉⡴⠃⠀⢀⡴⡆⠳⡤⠤⠆⡇⠀
⠀⠈⣏⠈⠉⢦⡀⠀⠀⠀⠀⠀⠙⠒⠈⠉⠛⡛⣫⠆⠀⢠⣾⣷⣷⠀⠀⠢⢠⠇⠀
⠀⠀⠘⣧⣄⠀⣩⠢⣄⠀⠀⠀⠠⠤⠴⠚⠉⠺⠃⠀⢀⡟⣿⠙⢿⢀⣄⣤⡞⠀⠀
⠀⠀⠀⠀⠙⢳⣬⠀⢼⣷⡀⢄⣤⣤⣴⣦⠴⠁⠀⠐⡜⣆⠸⣆⣘⢸⡇⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠈⠟⠀⠀⠙⣯⠉⠉⢒⣯⣿⠀⠀⠀⠀⠀⠈⠉⠙⠛⠈⡇⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⢸⡀⢀⣀⣈⣇⣴⣿⢏⣼⣦⡈⠑⠲⠤⣤⣀⣀⡠⠺⠇⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⢧⠀⠀⠉⠉⢻⣵⣿⣿⣿⣿⢷⢠⣤⣀⣈⣀⠈⠜⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠘⢣⡀⢀⡀⠀⠙⢿⣿⣿⢏⠎⣼⠋⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⢹⣆⠙⠢⣕⣤⠙⠓⢋⡜⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠑⠶⢦⠭⣽⡶⠋⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
 */
package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  public static Field2d field;
  private RobotContainer m_robotContainer;

  public Robot() {
    field = new Field2d();
    m_robotContainer = new RobotContainer();
    m_robotContainer.configureBindings();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putData("field", field);
    //field.setRobotPose(CameraUsing.robotPose2d);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    // Get the autonomous command from RobotContainer
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
