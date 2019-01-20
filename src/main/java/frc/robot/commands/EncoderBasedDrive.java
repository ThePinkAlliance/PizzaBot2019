/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import frc.robot.Robot;

public class EncoderBasedDrive extends Command implements PIDSource, PIDOutput {

  
  public static final double CMD_TOLERANCE        = 1.0;
  public static final double CMD_Kp               = 0.33;
  public static final double CMD_Ki               = 0.0;
  public static final double CMD_Kd               = 0.0;
  public static final double CMD_MAX_OUTPUT       = 1.0;
  public static final double CMD_MIN_OUTPUT       = 0.0;
  public static final double CMD_DEFAULT_DISTANCE = 30.0;

  private PIDSourceType pidSourceType = PIDSourceType.kDisplacement;
  private PIDController pidController = null;
  private double distance = 0.0;
  private double maxSpeed = CMD_MAX_OUTPUT;
  private Timer watchDogTimer = null;
  private double watchDogTime = 0;

  
  

  public EncoderBasedDrive(double distance, double completeCmdByThisTimeSecs, double maxSpeed) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_driveTrain);
    this.distance = distance;
    this.watchDogTime = completeCmdByThisTimeSecs;
    this.maxSpeed = maxSpeed;
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

    //Get the current PID values
    double P = Robot.m_driveTrain.getEncKp();
    double I = Robot.m_driveTrain.getEncKi();
    double D = Robot.m_driveTrain.getEncKd();

    //Setup the PID Controller to attempt drive by encoder
    setPIDSourceType(PIDSourceType.kDisplacement);  //distance traveled
    pidController = new PIDController(P, I, D, this, this);
    pidController.setName("EncoderBasedDrive");
    pidController.setContinuous(false);
    pidController.setAbsoluteTolerance(CMD_TOLERANCE);
    pidController.reset();  //resets previous error, disables controller per doc
    pidController.setSetpoint(distance);
    pidController.setOutputRange(CMD_MIN_OUTPUT, maxSpeed);
    Robot.m_driveTrain.resetEncoders();  //very important: start at zero.

    //GO
    pidController.enable();
    //Let the console know...
    System.out.println("EncoderBasedDrive:  enabled PIDController for distance of: " + distance);
    System.out.println("EncoderBasedDrive:  Kp: " + P + 
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

    double distanceTraveled = Robot.m_driveTrain.getFrontDistanceAverage();

    //Honor thy watchdog timer...
    double elapsedTime = watchDogTimer.get();
    if (elapsedTime > watchDogTime) {
      System.out.println("EncoderBasedDrive:  watchdog timer popped: " + 
                          distanceTraveled + "/" + distance);
      return true;
    }

    if (pidController.onTarget() || distanceTraveled > distance) {
      System.out.println("EncoderBasedDrive: onTarget or traveled far enough: " +
                         distanceTraveled + "/" + distance);
       return true;
    }

    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    pidController.disable();
    Robot.m_driveTrain.stopDriveTrain();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    pidController.disable();
    Robot.m_driveTrain.stopDriveTrain();
  }

  @Override 
  public void pidWrite(double output) {
     //write to drive train
     Robot.m_driveTrain.tankDriveByEncoder(output, output);    
  }

  @Override 
  public double pidGet() {
     //Get the amount left to target
     return Robot.m_driveTrain.getFrontDistanceAverage();
  }

  @Override
  public void setPIDSourceType(PIDSourceType pidSourceType) {
    this.pidSourceType = pidSourceType;
  }

  @Override
  public PIDSourceType getPIDSourceType() {
    return pidSourceType;
  }


}
