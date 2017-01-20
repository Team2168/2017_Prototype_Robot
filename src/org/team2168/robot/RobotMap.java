package org.team2168.robot;

import java.text.DecimalFormat;

import org.team2168.robot.PID.sensors.AverageEncoder;
import org.team2168.robot.utils.BNO055;

import edu.wpi.first.wpilibj.CounterBase;
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
	
	public static final int SHOOTER_MOTOR = 7;
	
	public static final int INTAKE_MOTOR_1 = 10;
	public static final int INTAKE_MOTOR_2 = 11;
	
	public static final int HOPPER_MOTOR = 9;
	
	
	/////////////DIO///////////////
	public static final int LEFT_DRIVE_ENCODER_A = 0;
	public static final int LEFT_DRIVE_ENCODER_B = 1;
	public static final int RIGHT_DRIVE_ENCODER_A = 2;
	public static final int RIGHT_DRIVE_ENCODER_B = 3;
	public static final int SHOOTER_ENCODER_A = 4;
	public static final int SHOOTER_ENCODER_B = 5;

	
	public static final boolean PRINT_SD_DEBUG_DATA = true;
	public static final long CONSOLE_PRINTER_LOG_RATE_MS = 100;
	
	/*************************************************************************
	 *                         Shooter PARAMETERS
	 *************************************************************************/
	//TODO check if the reverse values match the physical robot
	public static final boolean REVERSE_SHOOTER_WHEEL_FWD= false;
	public static final boolean REVERSE_SHOOTER_WHEEL_AFT= false;
	
	private static final int SHOOTER_PULSE_PER_ROTATION = 256; //encoder ticks per rotation
	//TODO find ratio
	private static final double SHOOTER_GEAR_RATIO = 14.0/28.0; //ratio between wheel over encoder
	private static final double SHOOTER_WHEEL_DIAMETER = 4;
	public static final int SHOOTER_ENCODER_PULSE_PER_ROT = (int) (SHOOTER_PULSE_PER_ROTATION * SHOOTER_GEAR_RATIO); //pulse per rotation * gear ratio
	public static final double SHOOTER_ENCODER_DIST_PER_TICK = (Math.PI * SHOOTER_WHEEL_DIAMETER / SHOOTER_ENCODER_PULSE_PER_ROT);
	public static final CounterBase.EncodingType SHOOTER_ENCODING_TYPE = CounterBase.EncodingType.k1X; //count only the rising edge
	public static final AverageEncoder.PositionReturnType SHOOTER_POS_RETURN_TYPE = AverageEncoder.PositionReturnType.FEET;
	public static final AverageEncoder.SpeedReturnType SHOOTER_SPEED_RETURN_TYPE = AverageEncoder.SpeedReturnType.RPM;
	public static final double SHOOTER_ENCODER_MIN_RATE = 0.1; //minimum inch per second
	public static final int SHOOTER_ENCODER_MIN_PERIOD = 1;
	public static final boolean SHOOTER_ENCODER_REVERSE = false;
	public static final int SHOOTER_AVG_ENCODER_VAL = 120;
	public static final double MIN_SHOOTER_SPEED = 0.2;
	public static final double SHOOTER_AUTO_NORMAL_SPEED = 0.5;
	public static final double SHOOTER_WHEEL_BASE = 2; //units must match PositionReturnType (feet)
	//TODO get correct values
	public static final double SHOOTER_BOULDER_STOP_VOLTAGE = 0.2;
	public static final double SHOOTER_CONSTANT_SPEED = 0.2;
	public static double CAMERA_OFFSET_ANGLE = 0; //degrees, camera in center of shooter
	
	
	/*************************************************************************
	 *                            PID PARAMETERS
	 *************************************************************************/
	//period to run PID loops on drive train
	public static final long DRIVE_TRAIN_PID_PERIOD = 20;//70ms loop
	public static final int DRIVE_TRAIN_PID_ARRAY_SIZE = 30;

	//PID Gains for Left & Right Speed and Position
	//Bandwidth =
	//Phase Margin =
	public static final double DRIVE_TRAIN_LEFT_SPEED_P =  0.4779;
	public static final double DRIVE_TRAIN_LEFT_SPEED_I =  1.0526;
	public static final double DRIVE_TRAIN_LEFT_SPEED_D =  0.0543;

	public static final double DRIVE_TRAIN_RIGHT_SPEED_P = 0.4779;
	public static final double DRIVE_TRAIN_RIGHT_SPEED_I = 1.0526;
	public static final double DRIVE_TRAIN_RIGHT_SPEED_D = 0.0543;

	public static final double DRIVE_TRAIN_LEFT_POSITION_P = 0.2;
	public static final double DRIVE_TRAIN_LEFT_POSITION_I = 0.0001412646174233;
	public static final double DRIVE_TRAIN_LEFT_POSITION_D = 0.0074778888124088;

	public static final double DRIVE_TRAIN_RIGHT_POSITION_P = 0.25;
	public static final double DRIVE_TRAIN_RIGHT_POSITION_I = 0.0001412646174233;
	public static final double DRIVE_TRAIN_RIGHT_POSITION_D = 0.0074778888124088;

	public static final double ROTATE_POSITION_P = 0.024;
	public static final double ROTATE_POSITION_I = 0.027;
	public static final double ROTATE_POSITION_D = 0.000000067;
	
	
	public static final double ROTATE_POSITION_P_CAM = 0.024;
	public static final double ROTATE_POSITION_I_CAM = 0.027;
	public static final double ROTATE_POSITION_D_CAM = 0.000000067;
	

	public static final double ROTATE_POSITION_CAMERA_MAX = 0.28;
	public static final double ROTATE_POSITION_CAMERA_MIN = 0.15;
	
	public static final double ROTATE_POSITION_P_Drive_Straight = 0.045;
	public static final double ROTATE_POSITION_I_Drive_Straight = 0.001;
	public static final double ROTATE_POSITION_D_Drive_Straight = 0.0064778888124088;

	//Shooter PID Speed
	//Bandwidth =
	//Phase Margin =
//	public static final double SHOOTER_SPEED_P = 0.000035;
//	public static final double SHOOTER_SPEED_I = 0.000053; 
//	public static final double SHOOTER_SPEED_D = 0.0000011838;
//	public static final double SHOOTER_SPEED_N = 6.8807;
	
	public static final double SHOOTER_SPEED_P = 0.000037;
	public static final double SHOOTER_SPEED_I = 0.000083; 
	public static final double SHOOTER_SPEED_D = 0.0000011838;
	public static final double SHOOTER_SPEED_N = 6.8807;
	
	/****************************************************************
	 *                TCP Servers  (ONLY FOR DEBUGGING)             *
	 ****************************************************************/
	public static final int TCP_SERVER_DRIVE_TRAIN_POS = 1180;
	public static final int TCP_SERVER_ROTATE_CONTROLLER = 1181;
	public static final int TCO_SERVER_RIGHT_DRIVE_TRAIN_SPEED = 1182;
	public static final int TCP_SERVER_LEFT_DRIVE_TRAIN_SPEED = 1183;
	public static final int TCP_SERVER_SHOOTER_SPEED = 1184;
	public static final int TCP_SERVER_ROTATE_CONTROLLER_STRAIGHT = 1185;
	public static final int TCP_SERVER_ROTATE_CAMERA_CONTROLLER = 1186;
	
	/*************************************************************************
	 *                            PDP PARAMETERS
	 *************************************************************************/
	public final static long PDPThreadPeriod = 20;
	public static final double WARNING_CURRENT_LIMIT = 20;
	public static final double STALL_CURRENT_LIMIT = 35;

	public final static int SHOOTER_MOTOR_FWD_PDP = 8;
	public final static int SHOOTER_MOTOR_AFT_PDP = 9;
	
}
