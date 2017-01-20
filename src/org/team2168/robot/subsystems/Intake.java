package org.team2168.robot.subsystems;

import org.team2168.robot.RobotMap;
import org.team2168.robot.commands.DriveIntakeWithConstant;
import org.team2168.robot.commands.DriveIntakeWithJoystick;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Subsystem class for the Intake
 */
public class Intake extends Subsystem {

	private static Spark intakeMotor;
	
	static Intake instance = null;
	
	public Intake() {
		intakeMotor = new Spark(RobotMap.INTAKE_MOTOR);
		
	}
	
	public static Intake getInstance() {
		if(instance == null)
		   instance = new Intake();
		
		return instance;
	}
	
	public void driveIntakeMotor1(double speed) {
		intakeMotor.set(speed);
	}

	/**
	 * 
	 * @param speed positive values shoot a ball out of the robot
	 */
	public void driveIntake(double speed) {
		driveIntakeMotor1(speed);
	}
	
    public void initDefaultCommand() {
        setDefaultCommand(new DriveIntakeWithJoystick());
    }
}

