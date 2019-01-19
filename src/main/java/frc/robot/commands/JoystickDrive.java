/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Robot;

public class JoystickDrive extends Command {

  private XboxController js = null; 
  
  /**
   * Constructor: needs require and sets the member js so that it can be used
   * through out the instance of the object.
   */
  public JoystickDrive() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_driveTrain);
    js = Robot.m_oi.getBaseJoystick();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    HandleButtons();
    HandleTankDrive();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }

  /** 
   * Perform tankdrive action during execute() calls
   */
  public void HandleTankDrive() {
    if (js != null) {
      double left = js.getY(Hand.kLeft);
      double right =  js.getY(Hand.kRight);
      Robot.m_driveTrain.tankDrive(left, right);
    }
  }
  
  /**
   * Perform any button actions during execute calls
   */
  public void HandleButtons() {
    if (js != null) {
      //Reset Encoders
      if (js.getXButtonPressed()) {
        Robot.m_driveTrain.resetEncoders();
      }
    }
  }
}