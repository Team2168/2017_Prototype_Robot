package org.team2168.robot.subsystems;

import org.team2168.robot.RobotMap;
import org.team2168.robot.commands.DriveHopperWithConstant;

import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Subsystem class for the Hopper
 */
public class Hopper extends Subsystem {

	private static Victor hopperMotor;
	
	static Hopper instance = null;
	
	public Hopper() {
		hopperMotor = new Victor(RobotMap.HOPPER_MOTOR);
	}
	
	public static Hopper getInstance(){
		if(instance == null)
			instance = new Hopper();
		
		return instance;
	}
	
	public void driveHopperMotor(double speed) {
		hopperMotor.set(speed);
	}

    public void initDefaultCommand() {
        setDefaultCommand(new DriveHopperWithConstant(0));
    }
}

