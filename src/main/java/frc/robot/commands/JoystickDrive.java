/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotDashboard;

public class JoystickDrive extends Command {

  private Joystick js = null; 
  
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
      double left = js.getRawAxis(OI.leftStick);//js.getY(Hand.kLeft);
      double right =  js.getRawAxis(OI.rightStick);//js.getY(Hand.kRight);
      //if (left > 0.3 || right > 0.3)
      //   System.out.println("LEFT: " + left + " RIGHT: " + right);
      Robot.m_driveTrain.tankDriveByJoystick(left, right);
    }
  }
  
  /**
   * Perform any button actions during execute calls
   */
  public void HandleButtons() {
    if (js != null) {
      //Reset Encoders
      if (js.getRawButtonPressed(OI.xButtonNumber)) {
        Robot.m_driveTrain.resetEncoders();
      }

      //Reset Gyro
      if (js.getRawButtonPressed(OI.bButtonNumber)) {
        Robot.m_driveTrain.resetGyro();
      }

      if (js.getRawButtonPressed(OI.aButtonNumber)) {
        EncoderBasedDrive2PIDController testCmd = new EncoderBasedDrive2PIDController(
          SmartDashboard.getNumber(RobotDashboard.DT_ENC_PID_DISTANCE, 30.0), 
          10, 
          SmartDashboard.getNumber(RobotDashboard.DT_ENC_PID_MAX_OUTPUT, 1.0));
        testCmd.start();
      }

      if (js.getRawButtonPressed(OI.yButtonNumber)) {
        DriveStraightByGyro testCmd = new DriveStraightByGyro(
          SmartDashboard.getNumber(RobotDashboard.DT_ENC_PID_DISTANCE, 30.0), 
          10, 
          SmartDashboard.getNumber(RobotDashboard.DT_ENC_PID_MAX_OUTPUT, 1.0));
        testCmd.start();
      }
    }
  }
}
