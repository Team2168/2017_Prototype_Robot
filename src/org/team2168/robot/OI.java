package org.team2168.robot;

import org.team2168.robot.commands.DriveShooter;
import org.team2168.robot.commands.IntakeBall;
import org.team2168.robot.utils.F310;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    //// CREATING BUTTONS
    // One type of button is a joystick button which is any button on a joystick.
    // You create one by telling it which joystick it's on and which button
    // number it is.
    // Joystick stick = new Joystick(port);
    // Button button = new JoystickButton(stick, buttonNumber);
    
    // There are a few additional built in buttons you can use. Additionally,
    // by subclassing Button you can create custom triggers and bind those to
    // commands the same as any other Button.
    
    //// TRIGGERING COMMANDS WITH BUTTONS
    // Once you have a button, it's trivial to bind it to a button in one of
    // three ways:
    
    // Start the command when the button is pressed and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenPressed(new ExampleCommand());
    
    // Run the command while the button is being held down and interrupt it once
    // the button is released.
    // button.whileHeld(new ExampleCommand());
    
    // Start the command when the button is released  and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenReleased(new ExampleCommand());
	private static OI instance = null;
	
	public static F310 driverJoystick = new F310(0);
	public static F310 operatorJoystick = new F310(1);

	/**
	 * Private constructor for singleton class which instantiates the OI object
	 */
	private OI() {
		operatorJoystick.ButtonA().whenPressed(new IntakeBall(0.7));
		operatorJoystick.ButtonY().whenPressed(new IntakeBall(-0.7));
		operatorJoystick.ButtonB().whenPressed(new IntakeBall(0.0));
		operatorJoystick.ButtonX().whenPressed(new DriveShooter(0.7));
	}

	/**
	 * Returns an instance of the Operator Interface.
	 * @return is the current OI object
	 */
	public static OI getInstance(){
		if(instance == null)
			instance = new OI();

		return instance;
	}
}

