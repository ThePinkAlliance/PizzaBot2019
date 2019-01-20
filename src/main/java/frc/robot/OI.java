/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
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

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());

  //============================================
	// 		JOYSTICK VARIABLES:  XBOX CONTROLLER
	//============================================
	public static int baseJoystickPort = 0;
  // Define all raw button numbers
	public static int aButtonNumber = 1;
	public static int bButtonNumber = 2;
	public static int xButtonNumber = 3;
	public static int yButtonNumber = 4;
	public static int leftBumperButtonNumber = 5;
	public static int rightBumperButtonNumber = 6;
	//public static int leftTriggerButtonNumber = 7;  //Logitech
	//public static int rightTriggerButtonNumber = 8; //Logitech
	public static int selectButtonNumber = 7;
	public static int startButtonNumber = 8;
	public static int leftJoystickButtonNumber = 9;
  public static int rightJoystickButtonNumber = 10;
  //Axis values
  public static int LXAxis = 0;
  public static int leftStick = 1; //aka LYAxis
  public static int leftTrigger = 2;
  public static int rightTrigger = 3;
  public static int RXAxis = 4;
  public static int rightStick = 5; //aka RYAxis

  //public Joystick base = new Joystick(baseJoystickPort);
  public Joystick base = null; 
  public Button testButton = null; 
	
	public OI() {

    try {

      //Setup your joystick
      base = new Joystick(baseJoystickPort);
      testButton = new JoystickButton(base, aButtonNumber);

      //Enable buttons / actions 
      setupBaseJoystick();

    } catch (Exception e) {
      System.out.println("Error setting up joystick.");
      System.out.println(e.toString());
    }

  }
  
  public void setupBaseJoystick() {
    if (base != null) {
       //testButton.whenPressed(new EncoderBasedDrive(30.0, 10.0, 0.6));
    }
  }
	
	public Joystick getBaseJoystick() {
		return base;
	}
}
