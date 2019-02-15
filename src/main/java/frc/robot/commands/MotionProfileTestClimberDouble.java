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
import frc.robot.subsystems.MotionProfileMotor;
import frc.robot.subsystems.utils.MotionProfileClimberDouble;
import frc.robot.subsystems.utils.MotionProfileExample;

public class MotionProfileTestClimberDouble extends Command {

  private Joystick js = null; 
  private MotionProfileClimberDouble mp = null;
  private String direction = MotionProfileClimber.DIRECTION_UP;
  private boolean movingUp = false;
  

  public MotionProfileTestClimberDouble(String direction) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_climber);
    this.direction = direction;
    if (direction.equalsIgnoreCase(MotionProfileClimber.DIRECTION_UP)) {
      movingUp = true;
    } else {
      movingUp = false;
    }
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    mp = Robot.m_climber.getMP();
    if (movingUp) {
      mp.setupTalon(true);
    } else {
      mp.setupTalon(false);
    }
    mp.reset();
    Robot.m_climber.resetEncoderPosition(0);
    mp.setMotionProfileMode();
    mp.startMotionProfile();
    System.out.println("MotionProfileTest(): initialized");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    mp.control(movingUp);
    mp.setMotionProfileMode();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    //boolean mpPressed = Robot.m_oi.getBaseJoystick().getRawButton(OI.aButtonNumber);
    //System.out.println("mpPressed: " + mpPressed);

    boolean bMPDone = mp.isMotionProfileDone();
    boolean bTop = Robot.m_climber.limitTopF();
    boolean bBottom = Robot.m_climber.limitBottomF();
    boolean bWall = Robot.m_climber.limitWallF();
    boolean bLimit = (movingUp ? bTop : bBottom);

    return (bMPDone || bLimit || bWall);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    mp.stopMotionProfile();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    mp.stopMotionProfile();
  }

}
