/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.ClimberDefault;
import frc.robot.subsystems.utils.MotionProfileClimberDouble;
import frc.robot.subsystems.utils.MotionProfileExample;


/**
 * Add your docs here.
 */
public class MotionProfileClimber extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  TalonSRX _talonFR = null;//new TalonSRX(4);
  TalonSRX _talonFL = null;//new TalonSRX(3);
  TalonSRX _talonRR = null;//new TalonSRX(4);
  TalonSRX _talonRL = null;//new TalonSRX(3);
  DigitalInput switchTopF = null;
  DigitalInput switchBottomF = null;
  DigitalInput switchWallF = null;

  public final static String LOCATION_LEFT = "LEFT";
  public final static String LOCATION_RIGHT = "RIGHT";
  public final static String LOCATION_FRONT = "FRONT";
  public final static String LOCATION_BACK = "BACK";
  public final static String DIRECTION_UP = "UP";
  public final static String DIRECTION_DOWN = "DOWN";

  public final static boolean SWITCH_CLOSED = true;
  public final static boolean SWITCH_OPEN = false;

  private String location = "";


  /** some example logic on how one can manage an MP */
  MotionProfileClimberDouble _example = null;//new MotionProfileExample(_talonM, null);
  
  public MotionProfileClimber(int frontLeft, 
                              int frontRight, 
                              int rearLeft, 
                              int rearRight, 
                              String location) {
    this.location = location;

    _talonFL = new TalonSRX(frontLeft);
    _example = new MotionProfileClimberDouble(_talonFL, null);
    switchTopF = new DigitalInput(9);
    switchBottomF = new DigitalInput(8);
    switchWallF = new DigitalInput(7);

  }

  


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new ClimberDefault());
  }

  public boolean limitTopF() {
    if (switchTopF.get()) {
      return SWITCH_OPEN;
    } else {
      return SWITCH_CLOSED;
    }
  }

  public boolean limitBottomF() {
    if (switchBottomF.get()) {
      return SWITCH_OPEN;
    } else {
      return SWITCH_CLOSED;
    }
  }

  public boolean limitWallF() {
    if (switchWallF.get()) {
      return SWITCH_OPEN;
    } else {
      return SWITCH_CLOSED;
    }
  }

  public void resetEncoderPosition(int position) {
    _talonFL.setSelectedSensorPosition(0);
  }

  public double getEncPosition() {
    return _talonFL.getSelectedSensorPosition();
  }

  public void set(double output) {
    _talonFL.set(ControlMode.PercentOutput, output);
  }

  public boolean isMotionProfileFinished() {
    return _example.isMotionProfileDone();
  }

  public MotionProfileClimberDouble getMP() {
    return _example;
  }

  public String getLocation() {
    return location;
  }
}
