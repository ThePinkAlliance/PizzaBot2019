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
import frc.robot.subsystems.MotionProfileMotor;
import frc.robot.subsystems.utils.MotionProfileExample;

public class MotionProfileTest2 extends Command {

  private Joystick js = null; 
  private MotionProfileExample mp = null;
  

  public MotionProfileTest2() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_motionR);
    

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    mp = Robot.m_motionR.getMP();
    if (Robot.m_motionR.getLocation().equalsIgnoreCase(MotionProfileMotor.LOCATION_LEFT)) {
      mp.setupTalon(true);
    } else {
      mp.setupTalon(false);
    }
    mp.reset();
    Robot.m_motionR.resetEncoderPosition(0);
    mp.setMotionProfileMode();
    mp.startMotionProfile();
    System.out.println("MotionProfileTest2(): initialized");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    mp.control();
    mp.setMotionProfileMode();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    boolean mpPressed = Robot.m_oi.getBaseJoystick().getRawButton(OI.aButtonNumber);
    //System.out.println("mpPressed: " + mpPressed);

    boolean bMPDone = mp.isMotionProfileDone();

    return (mpPressed || bMPDone);
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
