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

public class MotionProfileTest extends Command {

  private Joystick js = null; 

  public MotionProfileTest() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_motion);

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.m_motion.setMotionProfileMode();
    Robot.m_motion.startMotionProfile();
    System.out.println("MotionProfileTest(): initialized");
  }

  // Called repeatedly whe  n this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.m_motion.callMotionProfileControl();
    Robot.m_motion.setMotionProfileMode();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    boolean mpPressed = Robot.m_oi.getBaseJoystick().getRawButton(OI.aButtonNumber);
    //System.out.println("mpPressed: " + mpPressed);
    return mpPressed;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_motion.stopMotionProfile();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }

}
