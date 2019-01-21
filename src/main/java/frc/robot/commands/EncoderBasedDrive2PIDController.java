/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PIDController;
import frc.robot.Robot;
import frc.robot.subsystems.utils.DriveTrainPIDOutputLeft;
import frc.robot.subsystems.utils.DriveTrainPIDOutputRight;
import frc.robot.subsystems.utils.DriveTrainPIDSourceLeft;
import frc.robot.subsystems.utils.DriveTrainPIDSourceRight;

public class EncoderBasedDrive2PIDController extends Command {

  
  public static final double CMD_TOLERANCE        = 1.0;
  public static final double CMD_Kp               = 0.33;
  public static final double CMD_Ki               = 0.0;
  public static final double CMD_Kd               = 0.0;
  public static final double CMD_MAX_OUTPUT       = 1.0;
  public static final double CMD_MIN_OUTPUT       = 0.0;
  public static final double CMD_DEFAULT_DISTANCE = 30.0;

  private PIDController pidControllerL = null;
  private DriveTrainPIDOutputLeft pidOutputL = null;
  private DriveTrainPIDSourceLeft pidSourceL = null;

  private PIDController pidControllerR = null;
  private DriveTrainPIDOutputRight pidOutputR = null;
  private DriveTrainPIDSourceRight pidSourceR = null;

  private double distance = 0.0;
  private double maxSpeed = CMD_MAX_OUTPUT;
  private Timer watchDogTimer = null;
  private double watchDogTime = 0;

  
  

  public EncoderBasedDrive2PIDController(double distance, double completeCmdByThisTimeSecs, double maxSpeed) {
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

    //Get the current PID values (using one set of PID values for both sides - for now)
    double P = Robot.m_driveTrain.getEncKp();
    double I = Robot.m_driveTrain.getEncKi();
    double D = Robot.m_driveTrain.getEncKd();

    //Setup the PID Controller to attempt drive by encoder
    //setPIDSourceType(PIDSourceType.kDisplacement);  //distance traveled
    pidSourceL = new DriveTrainPIDSourceLeft();
    pidSourceL.setPIDSourceType(PIDSourceType.kDisplacement);
    pidOutputL = new DriveTrainPIDOutputLeft();
    pidControllerL = new PIDController(P, I, D, pidSourceL, pidOutputL);
    pidControllerL.setName("2PIDR");
    pidControllerL.setContinuous(false);
    pidControllerL.setAbsoluteTolerance(CMD_TOLERANCE);
    pidControllerL.reset();  //resets previous error, disables controller per doc
    pidControllerL.setSetpoint(distance);
    pidControllerL.setOutputRange(CMD_MIN_OUTPUT, maxSpeed);

    pidSourceR = new DriveTrainPIDSourceRight();
    pidSourceR.setPIDSourceType(PIDSourceType.kDisplacement);
    pidOutputR = new DriveTrainPIDOutputRight();
    pidControllerR = new PIDController(P, I, D, pidSourceR, pidOutputR);
    pidControllerR.setName("2PIDR");
    pidControllerR.setAbsoluteTolerance(CMD_TOLERANCE);
    pidControllerR.reset();  //resets previous error, disables controller per doc
    pidControllerR.setSetpoint(distance);
    pidControllerR.setOutputRange(CMD_MIN_OUTPUT, maxSpeed);


    Robot.m_driveTrain.resetEncoders();  //very important: start at zero.

    //GO
    pidControllerR.enable();
    pidControllerL.enable();

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

    double distanceTraveledL = Robot.m_driveTrain.getFrontLeftDistance();
    double distanceTraveledR = Robot.m_driveTrain.getFrontRightDistance();

    //Honor thy watchdog timer...
    double elapsedTime = watchDogTimer.get();
    if (elapsedTime > watchDogTime) {
      System.out.println("EncoderBasedDrive:  watchdog timer popped: " + 
                          distanceTraveledL + "/" + distanceTraveledR);
      return true;
    }

    boolean leftSideDone = (pidControllerL.onTarget() || distanceTraveledL > distance) ? true : false;
    boolean rightSideDone = (pidControllerR.onTarget() || distanceTraveledR > distance) ? true : false;
    
    //Ok, so in testing, pizzabot left side seems to finish first so we need to try to stop each motor as
    //soon as it reaches the setpoint without impacting the completion of the other motor.
    if (leftSideDone) {
      pidControllerL.disable();
      Robot.m_driveTrain.leftMotorStop();
      System.out.println("LEFT: onTarget or traveled far enough: " +
                         distanceTraveledL);
    } 

    if (rightSideDone) {
      pidControllerR.disable();
      Robot.m_driveTrain.rightMotorStop();
      System.out.println("RIGHT: onTarget or traveled far enough: " +
                         distanceTraveledR);
    }
    
    if (leftSideDone && rightSideDone) {
      System.out.println("DONE: onTarget or traveled far enough :" +
                         distanceTraveledL + "/" + distanceTraveledR);
       return true;
    }

    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    pidControllerL.disable();
    pidControllerR.disable();
    Robot.m_driveTrain.leftMotorStop();
    Robot.m_driveTrain.rightMotorStop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    pidControllerL.disable();
    pidControllerR.disable();
    Robot.m_driveTrain.leftMotorStop();
    Robot.m_driveTrain.rightMotorStop();
  }
}