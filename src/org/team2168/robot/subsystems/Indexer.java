package org.team2168.robot.subsystems;

import org.team2168.robot.RobotMap;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Indexer subsystem
 */
public class Indexer extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	private static Spark indexerMotor;
	
	private static Indexer instance = null;
	
	private Indexer() {
		indexerMotor = new Spark(RobotMap.INDEXER_MOTOR);
	}

	public static Indexer getInstance() {
		if(instance == null)
			instance = new Indexer();
		
		return instance;
	}
	
	public void setSpeed(double speed) {
		indexerMotor.set(speed);
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

