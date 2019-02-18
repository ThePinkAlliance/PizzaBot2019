/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.MotionProfileClimber;
import frc.robot.subsystems.utils.MotionProfileClimberDouble;

public class MotionProfileTestClimberDouble extends Command {

  private Joystick js = null; 
  private MotionProfileClimberDouble mp = null;
  private boolean movingUp = false;
  private Timer watchDog = null;
  private double watchDogTime = 0.0;
  private final double UNWIND_TIME = 0.0;  //one sec to let talon unwind
  private double doneTime = 0;
  private MotionProfileClimber climberPod = null;
  
  /**
   * 
   * @param direction which direction are we going UP or DOWN.  This affects
   *                  which motion profile is loaded
   * @param watchDogTime amount of time this command must complete in
   * 
   */
  public MotionProfileTestClimberDouble(MotionProfileClimber theClimberPod, String direction, double watchDogTime) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(theClimberPod);
    this.climberPod = theClimberPod;

    //set the direction boolean
    if (direction.equalsIgnoreCase(MotionProfileClimber.DIRECTION_UP)) {
      movingUp = true;
    } else {
      movingUp = false;
    }
    //cache your alloted time to complete this command
    this.watchDogTime = watchDogTime;
    //new up the timer for later use
    watchDog = new Timer();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    
    //prep your time
    watchDog.reset();
    watchDog.start();

    //get the motion profile object associated with the subsystem
    mp = climberPod.getMP(); //Robot.m_climber.getMP();
    if (movingUp) {
      climberPod.invertTalonFL(true);//Robot.m_climber.invertTalonFL(true);
    } else {
      climberPod.invertTalonFL(false);//Robot.m_climber.invertTalonFL(false);
    }
    mp.reset();
    climberPod.resetEncoderPosition(0);//Robot.m_climber.resetEncoderPosition(0);
    mp.setMotionProfileMode();
    //mp.startWorking(movingUp); //only used by threading alternative
    mp.startMotionProfile();
    System.out.println("MotionProfileTestClimberDouble(): initialized");
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
    double elapsedTime = watchDog.get();
    
    boolean bMPDone = mp.isMotionProfileDone();
    boolean bTop = climberPod.limitTop(); 
    boolean bBottom = climberPod.limitBottom();
    boolean bWall = climberPod.limitWall();
    boolean bLimit = (movingUp ? bTop : bBottom);

    if (elapsedTime >= watchDogTime) {
      System.out.println("Watch Dog timer popped.");
      return true;
    }

    if (bMPDone) {
      if (doneTime == 0) {
         doneTime = elapsedTime;
      }
      //let the unwind time occur
      double unwindElapsed = watchDog.get();
      if ((unwindElapsed - doneTime) < UNWIND_TIME) {
        bMPDone = false;
      }

    }

    return (bMPDone || bLimit || bWall);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    mp.stopMotionProfile();
    //mp.stopWorking();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    mp.stopMotionProfile();
   // mp.stopWorking();  //only used by threading alternative
  }

}
