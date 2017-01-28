package org.team2168.robot;

import java.text.DecimalFormat;

import org.team2168.robot.utils.BNO055;

import edu.wpi.first.wpilibj.Timer;
/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
    // For example to map the left and right motors, you could define the
    // following variables to use with your drivetrain subsystem.
    // public static int leftMotor = 1;
    // public static int rightMotor = 2;
    
    // If you are using multiple modules, make sure to define both the port
    // number and the module. For example you with a rangefinder:
    // public static int rangefinderPort = 1;
    // public static int rangefinderModule = 1;
	
	
	/////////////PWM//////////////
	public static final int LEFT_DRIVE_MOTOR_1 = 0;
	public static final int LEFT_DRIVE_MOTOR_2 = 1;
	public static final int LEFT_DRIVE_MOTOR_3 = 2;
	
	public static final int RIGHT_DRIVE_MOTOR_1 = 3;
	public static final int RIGHT_DRIVE_MOTOR_2 = 4;
	public static final int RIGHT_DRIVE_MOTOR_3 = 5;
	
	public static final int SHOOTER_MOTOR = 6;
	
	public static final int INTAKE_MOTOR_1 = 7;
	public static final int INTAKE_MOTOR_2 = 8;
	
	public static final int HOPPER_MOTOR = 9;
	
	public static final boolean INVERT_HOPPER_MOTOR = true;
	//public static final boolean INVERT_INTAKE_MOTOR_1 = true;
	//public static final boolean INVERT_INTAKE_MOTOR_2 = true;
	
	
	/////////////DIO///////////////
	public static final int LEFT_DRIVE_ENCODER_A = 0;
	public static final int LEFT_DRIVE_ENCODER_B = 1;
	public static final int RIGHT_DRIVE_ENCODER_A = 2;
	public static final int RIGHT_DRIVE_ENCODER_B = 3;
	public static final int SHOOTER_ENCODER_A = 4;
	public static final int SHOOTER_ENCODER_B = 5;

	
	public static final boolean PRINT_SD_DEBUG_DATA = true;
	public static final long CONSOLE_PRINTER_LOG_RATE_MS = 100;
	
}
