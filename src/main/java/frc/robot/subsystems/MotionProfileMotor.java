/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.MotionProfileTest;
import frc.robot.subsystems.utils.Constants;
import frc.robot.subsystems.utils.MotionProfileExample;


/**
 * Add your docs here.
 */
public class MotionProfileMotor extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  TalonSRX _talon = new TalonSRX(4);
  /** some example logic on how one can manage an MP */
  MotionProfileExample _example = new MotionProfileExample(_talon);
  
  public MotionProfileMotor() {
    
  }

  public void setupTalon() {
    /* Factory Default all hardware to prevent unexpected behaviour */
		_talon.configFactoryDefault();

		/* Configure Selected Sensor for Motion Profile */
		_talon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,
											Constants.kPIDLoopIdx,
											Constants.kTimeoutMs);
		/* Keep sensor and motor in phase, postive sensor values when MC LEDs are green */
		_talon.setSensorPhase(true);
		
		/**
		 * Configure MotorController Neutral Deadband, disable Motor Controller when
		 * requested Motor Output is too low to process
		 */
		_talon.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);

		/* Configure PID Gains, to be used with Motion Profile */
		_talon.config_kF(Constants.kPIDLoopIdx, Constants.kGains.kF, Constants.kTimeoutMs);
		_talon.config_kP(Constants.kPIDLoopIdx, Constants.kGains.kP, Constants.kTimeoutMs);
		_talon.config_kI(Constants.kPIDLoopIdx, Constants.kGains.kI, Constants.kTimeoutMs);
		_talon.config_kD(Constants.kPIDLoopIdx, Constants.kGains.kD, Constants.kTimeoutMs);

		/* Our profile uses 10ms timing */
		_talon.configMotionProfileTrajectoryPeriod(10, Constants.kTimeoutMs); 
		
		/* Status 10 provides the trajectory target for motion profile AND motion magic */
		_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);
  }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    //setDefaultCommand(new MotionProfileTest());
  }

  public void stopMotionProfile() {
    _talon.set(ControlMode.PercentOutput, 0);
    _example.reset();
  }

  public void resetMotionProfile() {
    _example.reset();
    
  }

  public void setMotionProfileMode() {
    SetValueMotionProfile setOutput = _example.getSetValue();
   	_talon.set(ControlMode.MotionProfile, setOutput.value);
  }

  public void callMotionProfileControl() {
    _example.control();
  }

  public void startMotionProfile() {
    _example.startMotionProfile();
  }

  public void set(double output) {
    _talon.set(ControlMode.PercentOutput, output);
  }
}
