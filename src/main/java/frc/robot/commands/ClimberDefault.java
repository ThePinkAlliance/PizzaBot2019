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

  public ClimberDefault() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_climber);
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
        Robot.m_climber.resetEncoderPosition(0);
      }

        //double left = js.getRawAxis(OI.leftStick);//js.getY(Hand.kLeft);
      double right =  js.getRawAxis(OI.rightStick);//js.getY(Hand.kRight);
      //if (left > 0.3 || right > 0.3)
      //   System.out.println("LEFT: " + left + " RIGHT: " + right);
      //System.out.println(right * 0.25 + " value from joystick");
      if (right > 0.0) {
        if (Robot.m_climber.limitBottomF() == MotionProfileClimber.SWITCH_CLOSED)
           right = 0;
      } else {
        if (Robot.m_climber.limitTopF() == MotionProfileClimber.SWITCH_CLOSED)
           right = 0;
      }
      //System.out.println(right * 1.0 + " value from joystick");
      Robot.m_climber.set(right * 1.0);

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
