package org.team2168.robot.subsystems;

import org.team2168.RobotMap;
import org.team2168.robot.subsystems.Drivetrain;
import org.team2168.robot.utils.consoleprinter.ConsolePrinter;
import org.team2168.robot.commands.DriveWithJoystick;

import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Subsystem class for the Drivetrain
 */
public class Drivetrain extends Subsystem {
	
	private static Victor LeftMotor1;
	private static Victor LeftMotor2;
	private static Victor LeftMotor3;
	
	private static Victor RightMotor1;
	private static Victor RightMotor2;
	private static Victor RightMotor3;
	
	private static Encoder RightEncoder;
	private static Encoder LeftEncoder;
	
	private static Drivetrain instance = null;
	
	private Drivetrain() {
		LeftMotor1 = new Victor(RobotMap.LEFT_DRIVE_MOTOR_1);
		LeftMotor2 = new Victor(RobotMap.LEFT_DRIVE_MOTOR_2);
		LeftMotor3 = new Victor(RobotMap.LEFT_DRIVE_MOTOR_3);
		
		RightMotor1 = new Victor(RobotMap.RIGHT_DRIVE_MOTOR_1);
		RightMotor2 = new Victor(RobotMap.RIGHT_DRIVE_MOTOR_2);
		RightMotor3 = new Victor(RobotMap.RIGHT_DRIVE_MOTOR_3);
		
		RightEncoder = new Encoder(RobotMap.RIGHT_DRIVE_ENCODER_A, RobotMap.RIGHT_DRIVE_ENCODER_B);
		LeftEncoder = new Encoder(RobotMap.LEFT_DRIVE_ENCODER_A, RobotMap.LEFT_DRIVE_ENCODER_B);
		
		//Log sensor data
		ConsolePrinter.putNumber("Drivetrain Right Encoder", Drivetrain::getRightEncoder, true, false);
		ConsolePrinter.putNumber("Drivetrain Left Encoder", Drivetrain::getLeftEncoder, true, false);
	}
	
	public static Drivetrain getInstance(){
		if(instance == null)
			instance = new Drivetrain();
		
		return instance;
	}
    
    public void driveLeftMotor1(double speed) {
    	LeftMotor1.set(speed);
    }
    
    public void driveLeftMotor2(double speed) {
    	LeftMotor2.set(speed);
    }
    
    public void driveLeftMotor3(double speed) {
    	LeftMotor3.set(speed);
    }
    
    public void driveLeftSide(double speed) {
    	driveLeftMotor1(speed);
    	driveLeftMotor2(speed);
    	driveLeftMotor3(speed);
    }
    
    public void driveRightMotor1(double speed) {
    	RightMotor1.set(speed);
    }
    
    public void driveRightMotor2(double speed) {
    	RightMotor2.set(speed);
    }
    
    public void driveRightMotor3(double speed) {
    	RightMotor3.set(speed);
    }
    
    public void driveRightSide(double speed) {
    	driveRightMotor1(speed);
    	driveRightMotor2(speed);
    	driveRightMotor3(speed);
    }
    
    public void driveRobot(double leftSpeed, double rightSpeed) {
    	driveLeftSide(leftSpeed);
    	driveRightSide(rightSpeed);
    }

    public static double getLeftEncoder() {
    	return LeftEncoder.get();
    }
    
    public static double getRightEncoder() {
    	return RightEncoder.get();
    }
    
    public void initDefaultCommand() {
        setDefaultCommand(new DriveWithJoystick());
    }
    
    
}

