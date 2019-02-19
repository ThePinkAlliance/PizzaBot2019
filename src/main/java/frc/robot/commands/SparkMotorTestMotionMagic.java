/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class SparkMotorTestMotionMagic extends Command {

  private double setPoint, processVariable;
  private boolean mode;

  public SparkMotorTestMotionMagic() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_spark);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    
    //mode = SmartDashboard.getBoolean("Mode", false);
    Robot.m_spark.setEncoderPosition(0);
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    mode = SmartDashboard.getBoolean("Mode", false);
    if(mode) {
      setPoint = SmartDashboard.getNumber("Set Velocity", 0);
      Robot.m_spark.getPIDController().setReference(setPoint, ControlType.kVelocity);
      processVariable = Robot.m_spark.getEncoder().getVelocity();
    } else {
      setPoint = SmartDashboard.getNumber("Set Position", 0);
      /**
       * As with other PID modes, Smart Motion is set by calling the
       * setReference method on an existing pid object and setting
       * the control type to kSmartMotion
       */
      Robot.m_spark.getPIDController().setReference(setPoint, ControlType.kSmartMotion);
      processVariable = Robot.m_spark.getEncoder().getPosition();
    }
    SmartDashboard.putNumber("SetPoint", setPoint);
    SmartDashboard.putNumber("Process Variable", processVariable);
    SmartDashboard.putNumber("Output", Robot.m_spark.getMotor().getAppliedOutput());
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
