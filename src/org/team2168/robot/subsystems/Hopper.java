package org.team2168.robot.subsystems;

import org.team2168.robot.RobotMap;
import org.team2168.robot.commands.DriveHopper;

import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Subsystem class for the Hopper
 */
public class Hopper extends Subsystem {

	private static Victor hopperMotor;
	private static Hopper instance = null;
	
	
	public Hopper() {
		hopperMotor = new Victor(RobotMap.HOPPER_MOTOR);
	}
	
	public static Hopper getInstance(){
		if(instance == null)
			instance = new Hopper();
		
		return instance;
	}
	
	public void driveHopperMotor(double speed) {
		if(RobotMap.INVERT_HOPPER_MOTOR)
			speed = speed * -1.0;
		
		hopperMotor.set(speed);
	}

    public void initDefaultCommand() {
        setDefaultCommand(new DriveHopper(0));
    }
}

