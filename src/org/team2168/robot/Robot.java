
package org.team2168.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

import org.team2168.robot.subsystems.Drivetrain;
import org.team2168.robot.subsystems.Hopper;
import org.team2168.robot.subsystems.Intake;
import org.team2168.robot.subsystems.Shooter;
import org.team2168.robot.utils.consoleprinter.ConsolePrinter;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	public static OI oi;
	public static Drivetrain drivetrain;
	public static Intake intake;
	public static Hopper hopper;
	public static Shooter shooter;

    Command autonomousCommand;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	ConsolePrinter.init();
    	ConsolePrinter.setRate(RobotMap.CONSOLE_PRINTER_LOG_RATE_MS);
    	
        // instantiate the command used for the autonomous period
        drivetrain = new Drivetrain();
        intake = new Intake();
        hopper = new Hopper();
        shooter = new Shooter();
        
        oi = OI.getInstance();
        
		ConsolePrinter.startThread();
    }
	
	public void disabledPeriodic() {
		// Kill all active commands
		Scheduler.getInstance().run();
	}

    public void autonomousInit() {
        // schedule the autonomous command (example)
        if (autonomousCommand != null) autonomousCommand.start();
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
    }

    public void teleopInit() {
		// This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to 
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) autonomousCommand.cancel();
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
        LiveWindow.run();
    }
}
