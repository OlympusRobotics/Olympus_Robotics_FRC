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

import java.io.File;
import java.util.Arrays;
import java.util.Comparator;

import com.ctre.phoenix6.HootAutoReplay;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import com.bionanomics.refinery.mcp.RoboRioMcpServer;

public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;
    private final Field2d field;

    /** Maximum number of log sessions to keep on the roboRIO. */
    private static final int MAX_LOG_SESSIONS = 20;
    private static final File LOG_DIR = new File("/home/lvuser/logs");

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        Logger.recordMetadata("ProjectName", "Theseus");
        Logger.recordMetadata("TeamNumber", "4982");

        if (isReal()) {
            Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs"));
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
        if (isReal()) {
            pruneOldLogs();
        }
        RoboRioMcpServer.start();
    }

    /**
     * Deletes the oldest log directories and .revlog files so only the most
     * recent {@link #MAX_LOG_SESSIONS} of each are kept on the roboRIO.
     */
    private void pruneOldLogs() {
        if (!LOG_DIR.isDirectory()) return;

        // Prune old session directories (CTRE hoot / AdvantageKit wpilog)
        File[] dirs = LOG_DIR.listFiles(File::isDirectory);
        if (dirs != null && dirs.length > MAX_LOG_SESSIONS) {
            Arrays.sort(dirs, Comparator.comparing(File::getName));
            int toDelete = dirs.length - MAX_LOG_SESSIONS;
            int deleted = 0;
            for (int i = 0; i < toDelete; i++) {
                if (deleteRecursive(dirs[i])) deleted++;
            }
            System.out.println("[LogRotate] Pruned " + deleted + " old log sessions, kept " + MAX_LOG_SESSIONS);
        }

        // Prune old .wpilog files
        pruneFilesByExtension(".wpilog");

        // Prune old REV .revlog files
        pruneFilesByExtension(".revlog");
    }

    private void pruneFilesByExtension(String ext) {
        File[] files = LOG_DIR.listFiles((dir, name) -> name.endsWith(ext));
        if (files != null && files.length > MAX_LOG_SESSIONS) {
            Arrays.sort(files, Comparator.comparing(File::getName));
            int toDelete = files.length - MAX_LOG_SESSIONS;
            int deleted = 0;
            for (int i = 0; i < toDelete; i++) {
                if (files[i].delete()) deleted++;
            }
            System.out.println("[LogRotate] Pruned " + deleted + " old " + ext + " files, kept " + MAX_LOG_SESSIONS);
        }
    }

    private boolean deleteRecursive(File file) {
        if (file.isDirectory()) {
            File[] children = file.listFiles();
            if (children != null) {
                for (File child : children) {
                    deleteRecursive(child);
                }
            }
        }
        return file.delete();
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
