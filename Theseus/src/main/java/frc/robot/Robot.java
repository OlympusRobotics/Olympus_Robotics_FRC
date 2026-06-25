package frc.robot;

//this is the file that the roborio reads, try to keep it clean

import java.io.File;
import java.util.Arrays;
import java.util.Comparator;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.SignalLogger;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


public class Robot extends LoggedRobot {
    public Command m_autonomousCommand;

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

        m_robotContainer = new RobotContainer(); //defines the robotContainer as a new object, the robotcontainer is where all of the other code is run. This single line is basically what tells the code to run
        field = new Field2d(); //creates a new field for for smartdashboard
    }

    @Override
    public void robotInit() {
        SignalLogger.start(); //begins logging data on the roborio, basically just motor movements and stuff
        if (isReal()) {
            pruneOldLogs(); //keeps the roborio from being full
        }
        // The refinery MCP library may be provided by a local composite build
        // or external dependency. In case the library is not present in the
        // current clone (e.g. the `refinery-roborio-mcp` folder is empty),
        // attempt to start it via reflection so the project can compile and
        // run without a hard dependency at compile time.
        try {
            Class<?> cls = Class.forName("com.bionanomics.refinery.mcp.RoboRioMcpServer");
            java.lang.reflect.Method m = cls.getMethod("start");
            m.invoke(null);
        } catch (ClassNotFoundException ex) {
            // Library not present; nothing to start.
        } catch (Exception ex) {
            ex.printStackTrace();
        }
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
    public void robotPeriodic() { //runs the entire time that the robot is running
        m_timeAndJoystickReplay.update();
        m_robotContainer.mcpJoystick.poll();
        CommandScheduler.getInstance().run(); //this command allows all of our commands in robotcontainer to run
        SmartDashboard.putData("Field", field); //updates the field on smartdashboard
        field.setRobotPose(m_robotContainer.drivetrain.getState().Pose); //updates the robot on the field in smartdashboard
    }

    @Override
    public void disabledInit() { //runs immediately upon entering disabled mode
        SignalLogger.stop();
        //m_robotContainer.intake.m_inkMot.setPosition(0);
    }

    @Override
    public void disabledPeriodic() { //runs continuously while disabled. If there was time LED code would be in here
    }

    @Override
    public void disabledExit() { //runs upon switching away from disabled
    }

    @Override
    public void autonomousInit() { //runs immediately upon entering autonomous mode
        m_autonomousCommand = m_robotContainer.getAutonomousCommand(); //sets the auto to what is selected

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand); //this is the line that actually runs it
        } 
        else {
            System.out.println("No autonomous command found!");
        }
    }

    @Override
    public void autonomousPeriodic() { //would run during autonomous mode
    }

    @Override
    public void autonomousExit() { //would run upon leaving autonomous mode
    }

    @Override
    public void teleopInit() { //runs immediately upon entering teleop mode
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand); //ends auto even if it is in the middle of a command
        }

        m_robotContainer.intake.m_inkMot.setPosition(0); //this is very redundant and unnecessary but this would change the encoder value of the intake motor to be 0 upon entering teleop, better hope it is actually at 0 when it switches!
    }

    @Override
    public void teleopPeriodic() { //would run while in teleop
    }

    @Override
    public void teleopExit() { //runs upon leaving teleop mode
    }

    @Override
    public void testInit() { //runs upon entering test mode
        CommandScheduler.getInstance().cancelAll(); //stops everything currently running
    }

    @Override
    public void testPeriodic() { //would run during test mode, good for small debugging things
    }

    @Override
    public void testExit() {//would run upon leaving test mode
    }

    @Override
    public void simulationPeriodic() { //would run while in simulation mode
    }
}
