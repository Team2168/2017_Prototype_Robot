package org.team2168.robot.subsystems;

import org.team2168.robot.RobotMap;
import org.team2168.robot.utils.consoleprinter.ConsolePrinter;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Subsystem class for the Shooter
 */
public class Shooter extends Subsystem {

	private static Victor shooterMotor;
	private static Encoder shooterEncoder;
	static Shooter instance = null;
	
	public Shooter() {
		shooterMotor = new Victor(RobotMap.SHOOTER_MOTOR);
		shooterEncoder = new Encoder(RobotMap.SHOOTER_ENCODER_A, RobotMap.SHOOTER_ENCODER_B);
		
		//Log sensor data
		ConsolePrinter.putNumber("Shooter Encoder", Shooter::getShooterEncoder, true, false);
	}
	
	public static Shooter getInstance(){
		if(instance == null)
			instance = new Shooter();
		
		return instance;
	}
	
	public void driveShooterMotor(double speed) {
		shooterMotor.set(speed);
	}
	
	public static double getShooterEncoder() {
		return shooterEncoder.get();
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

