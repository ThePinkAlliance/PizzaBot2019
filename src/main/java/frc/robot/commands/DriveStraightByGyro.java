/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.*;

public class DriveStraightByGyro extends Command implements PIDOutput {

  public static final String MYNAME               = "DriveStraightByGyro";
  public static final double CMD_TOLERANCE        = 1.0;
  public static final double CMD_Kp               = 0.02;
  public static final double CMD_Ki               = 0.0;
  public static final double CMD_Kd               = 0.0;
  public static final double CMD_MAX_OUTPUT       = 0.3;   
  public static final double CMD_MIN_OUTPUT       = 0.0;
  public static final double CMD_DEFAULT_ANGLE    = 0.0;
  private PIDController turnController = null;
  

  private Timer watchDogTimer = null;
  private double watchDogTime = 0;

  private double desiredDistance = 0;
  private double desiredAngle = CMD_DEFAULT_ANGLE;
  private double maxSpeed = CMD_MAX_OUTPUT;

  public DriveStraightByGyro(double desiredDistance, double completeCmdByThisTimeSecs, double maxSpeed) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_driveTrain);

    //set up requirements
    this.desiredAngle = CMD_DEFAULT_ANGLE;
    this.maxSpeed = maxSpeed;
    this.watchDogTime = completeCmdByThisTimeSecs;
    this.desiredDistance = desiredDistance;

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
      //The command must complete before the timer pops.  Once the timer pops the
      //isFinished condition will be forced to true whether the command completed
      //or not!
      watchDogTimer = new Timer();
      watchDogTimer.reset();
      watchDogTimer.start();

      //Get the current PID values (using one set of PID values for both sides - for now)
      double P = Robot.m_driveTrain.getNavxKp();
      double I = Robot.m_driveTrain.getNavxKi();
      double D = Robot.m_driveTrain.getNavxKd();

      
      turnController = new PIDController(P, 
                                         I,
                                         D,
                                         Robot.m_driveTrain.getNavxPIDSource(), this);
      turnController.setInputRange(-180.0f,  180.0f);
      turnController.setOutputRange(-1.0, 1.0);
      turnController.setAbsoluteTolerance(CMD_TOLERANCE);
      turnController.setContinuous(true);
      turnController.disable();
      turnController.setSetpoint(desiredAngle);
      Robot.m_driveTrain.resetGyro();
      Robot.m_driveTrain.resetEncoders();
      turnController.enable();

      //Let the console know...
      System.out.println(MYNAME + ":  enabled PIDController for distance of: " + 
                                     desiredDistance + " speed: " + maxSpeed );
      System.out.println(MYNAME + ":  Kp: " + P + 
                                    " Ki: " + I + 
                                    " Kd: " + D);

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    
    double currentDistance = Robot.m_driveTrain.getFrontDistanceAverage();
        
    //Honor thy watchdog timer...
    double elapsedTime = watchDogTimer.get();
    if (elapsedTime > watchDogTime) {
      System.out.println(MYNAME + ": watchdog timer popped: distance" + currentDistance);
      return true;
    //Check distance traveled...
    } else if (currentDistance >= desiredDistance) {  //GO STRAIGHT
      System.out.println(MYNAME + ": distance reached: " + currentDistance);
      turnController.disable();
      Robot.m_driveTrain.stopDriveTrain();
      return true;
    //Keep going
    } else  {
      return false;
    }
    
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    turnController.disable();
    turnController.close();
    Robot.m_driveTrain.stopDriveTrain();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    turnController.disable();
    turnController.close();
    Robot.m_driveTrain.stopDriveTrain();
  }

  @Override 
  public void pidWrite(double output) {
    double left = maxSpeed;
    double right = maxSpeed;

    if (output > 0.0)
    {
      right = right + output;
      left = left - output;
    } else {
      right = right - output;
      left = left + output;
    }
    //write to drive train
    System.out.println(MYNAME + ": OUTPUT: " + output + " LEFT: " + left + " RIGHT: " + right);
    Robot.m_driveTrain.tankDriveByEncoder(left, right);    
  }
}
