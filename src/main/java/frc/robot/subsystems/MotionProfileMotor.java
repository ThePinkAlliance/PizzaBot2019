/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

import frc.robot.subsystems.utils.MotionProfileExample;


/**
 * Add your docs here.
 */
public class MotionProfileMotor extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  TalonSRX _talonM = null;//new TalonSRX(4);
  TalonSRX _talonS = null;//new TalonSRX(3);

  public final static String LOCATION_LEFT = "LEFT";
  public final static String LOCATION_RIGHT = "RIGHT";
  public final static String LOCATION_FRONT = "FRONT";
  public final static String LOCATION_BACK = "BACK";

  private String location = "";


  /** some example logic on how one can manage an MP */
  MotionProfileExample _example = null;//new MotionProfileExample(_talonM, null);
  
  public MotionProfileMotor(int mId, int sId, String location) {
    this.location = location;
    _talonM = new TalonSRX(mId);
    _talonS = new TalonSRX(sId);
    _example = new MotionProfileExample(_talonM, _talonS);
  }

  


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    //setDefaultCommand(new MotionProfileTest());
  }

  

  public void resetEncoderPosition(int position) {
    _talonM.setSelectedSensorPosition(0);
  }

  public double getEncPosition() {
    return _talonM.getSelectedSensorPosition();
  }

  public void set(double output) {
    _talonM.set(ControlMode.PercentOutput, output);
  }

  public boolean isMotionProfileFinished() {
    return _example.isMotionProfileDone();
  }

  public MotionProfileExample getMP() {
    return _example;
  }

  public String getLocation() {
    return location;
  }
}
