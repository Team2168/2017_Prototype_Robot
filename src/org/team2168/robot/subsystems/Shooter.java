package org.team2168.robot.subsystems;

import org.team2168.robot.Robot;
import org.team2168.robot.RobotMap;
import org.team2168.robot.PID.controllers.PIDSpeed;
import org.team2168.robot.PID.sensors.AverageCounter;
import org.team2168.robot.PID.sensors.AverageEncoder;
import org.team2168.robot.commands.DriveShooterWithJoysticks;
import org.team2168.robot.utils.TCPSocketSender;
import org.team2168.robot.utils.Util;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Shooter Subsystem 
 * @author Krystina
 */
public class Shooter extends Subsystem {
	private Spark shooterFWD;
	private Spark shooterAFT;
	private AverageEncoder shooterEncoder;
	private AnalogInput shooterDistanceSensor;
	
	//TODO calibrate values
	private final double MIN_SENSOR_VOLTAGE = 0.6;
	private final double BOULDER_PRESENT_VOLTAGE = 1.5;
	private final double IR_SENSOR_AVG_GAIN = 0.5;
	private double averagedBoulderDistance = 0.0;
	
	static Shooter instance = null;
	
	//declare speed controllers
	public PIDSpeed shooterSpeedController;
	
	//declare TCP severs...ONLY FOR DEBUGGING PURPOSES, SHOULD BE REMOVED FOR COMPITITION
	TCPSocketSender TCPShooterController;
	
	//output voltage...ONLY FOR DEBUGGING PURPOSES, SHOULD BE REMOVED FOR COMPITITION
	private volatile double AFTMotorVoltage;
	private volatile double FWDMotorVoltage;
		
	/**
	 * Private singleton constructor for the Shooter subsystem
	 */
	private Shooter () {
		
		
			shooterFWD = new Spark (RobotMap.SHOOTER_WHEEL_FWD);
			shooterFWD.setExpiration(0.1);
			shooterFWD.setSafetyEnabled(true);
			
			shooterAFT = new Spark (RobotMap.SHOOTER_WHEEL_AFT);
			shooterAFT.setExpiration(0.1);
			shooterAFT.setSafetyEnabled(true);
		

		shooterEncoder = new AverageEncoder(RobotMap.SHOOTER_ENCODER_A, 
				   							   RobotMap.SHOOTER_ENCODER_B, //uncomment for encoder
				   							   RobotMap.SHOOTER_ENCODER_PULSE_PER_ROT,
				   							   RobotMap.SHOOTER_ENCODER_DIST_PER_TICK,
				   							   RobotMap.SHOOTER_ENCODER_REVERSE,
				   							   RobotMap.SHOOTER_ENCODING_TYPE, //uncomment for encoder
				   							   RobotMap.SHOOTER_SPEED_RETURN_TYPE,
				   							   RobotMap.SHOOTER_POS_RETURN_TYPE,
				   							   RobotMap.SHOOTER_AVG_ENCODER_VAL);
		
		shooterEncoder.setMinRate(RobotMap.SHOOTER_ENCODER_MIN_RATE);
		
		
		//Spawn new PID Controller
		shooterSpeedController = new PIDSpeed(
				"ShooterSpeedController",
				RobotMap.SHOOTER_SPEED_P,
				RobotMap.SHOOTER_SPEED_I,
				RobotMap.SHOOTER_SPEED_D,
				RobotMap.SHOOTER_SPEED_N,
				shooterEncoder,
				RobotMap.DRIVE_TRAIN_PID_PERIOD);
		
		shooterSpeedController.setSIZE(RobotMap.DRIVE_TRAIN_PID_ARRAY_SIZE);

		//start controller threads
		shooterSpeedController.startThread();
		
		
		TCPShooterController = new TCPSocketSender(RobotMap.TCP_SERVER_SHOOTER_SPEED, shooterSpeedController);
		TCPShooterController.start();
		
		AFTMotorVoltage = 0;
		FWDMotorVoltage = 0;
	}
	
	/**
	 * singleton object for Shooter_Superman
	 * @return rerturns the shooter singleton object
	 * @author Krystina
	 */
	public static Shooter getInstance() {
		if (instance == null)
			instance = new Shooter();
		
		return instance;
	}
	
	/**
	 * Takes in a speed and drives the Shooter_Superman wheels in the same directions
	 * @param speed -1 to 1 if given a positive value the ball will move inward. If negative the ball will move outward.
	 * @author Krystina
	 */
	public void driveShooter(double speed) {
		driveFWDShooterWheel(speed);
		driveAFTShooterWheel(speed);
		
	}
	
	/**
	 * Takes in a speed and drives the first shooter wheel in a positive or inward direction
	 * @param speed -1 to 1
	 * @author Krystina
	 */
	public void driveFWDShooterWheel(double speed) {
		if(RobotMap.REVERSE_SHOOTER_WHEEL_FWD)
			speed = -speed;
		
		shooterFWD.set(speed);
		FWDMotorVoltage = Robot.pdp.getBatteryVoltage() * speed;
	}
	
	/**
	 * Takes in a speed and drives the second shooter wheel in a positive or inward direction
	 * @param speed -1 to 1
	 * @author Krystina
	 */
	public void driveAFTShooterWheel(double speed) {
		if(RobotMap.REVERSE_SHOOTER_WHEEL_AFT)
			speed = -speed;
			
		shooterAFT.set(speed);
		AFTMotorVoltage = Robot.pdp.getBatteryVoltage() * speed;
	}
	
	/**
	 * Gets the speed of the shooter wheel
	 * @return speed in RPM
	 */
	public double getSpeed() {
		return shooterEncoder.getRate();
	}
	
	
	/**
	 * zeros the position traveled by motors
	 */
	public void resetPosition() {
		shooterEncoder.reset();
	}
	/**
	 * Returns the last commanded voltage to the motor
	 * @return double in volts representing last commanded voltage to motor
	 */
	public double getAFTMotorVoltage() {
		return AFTMotorVoltage;
	}

	/**
	 * Returns the last commanded voltage to the motor
	 * @return double in volts representing last commanded voltage to motor
	 */
	public double getFWDMotorVoltage() {
		return FWDMotorVoltage;
	}
	
	/**
	 * Returns true if boulder is within range of the sensor
	 * @return boolean true if boulder is present
	 */
	public boolean isBoulderPresent() {
		return Robot.shooter.getAveragedRawBoulderDistance() > BOULDER_PRESENT_VOLTAGE;
	}
	
	/**
	 * Returns the raw voltage from the shooter distance sensor
	 * @return double voltage
	 */
	public double getRawBoulderDistance() {
		return Util.max(MIN_SENSOR_VOLTAGE, shooterDistanceSensor.getVoltage());
	}
	
	/**
	 * Note, this method should be called from a loop to prevent data from getting stale.
	 * @return the average voltage from the shooter distance sensor
	 */
	public double getAveragedRawBoulderDistance() {
		averagedBoulderDistance = Util.runningAverage(getRawBoulderDistance(),
				averagedBoulderDistance, IR_SENSOR_AVG_GAIN);
		return averagedBoulderDistance;
	}
	
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
    	setDefaultCommand(new DriveShooterWithJoysticks());
    }

	public double getPosition() {
		// TODO Auto-generated method stub
		return shooterEncoder.getPos();
	}
}

