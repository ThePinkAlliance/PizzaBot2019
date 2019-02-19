/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.MotionProfileClimber;

public class ClimberDefault extends Command {

  private Joystick js = null; 
  private MotionProfileClimber climberPod = null;

  public ClimberDefault(MotionProfileClimber theClimberPod) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(theClimberPod);
    this.climberPod = theClimberPod;
    js = Robot.m_oi.getBaseJoystick();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (js != null) {

      if (js.getRawButtonPressed(OI.aButtonNumber)) {
        climberPod.resetEncoderPosition(0);
      }

      double value =  js.getRawAxis(OI.rightStick);
      /**
       * YOU MUST CHECK JS VALUE and verify whether positive is down / negative is up
       */
      if (value > 0.0) {
        if (climberPod.limitBottom() == MotionProfileClimber.SWITCH_CLOSED)
           value = 0;
      } else {
        if (climberPod.limitTop() == MotionProfileClimber.SWITCH_CLOSED)
           value = 0;
      }
      //System.out.println(value + " value from joystick");
      climberPod.set(value);

    }
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
}
