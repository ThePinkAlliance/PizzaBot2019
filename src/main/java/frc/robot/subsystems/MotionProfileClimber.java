/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.subsystems.utils.Constants;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.ClimberDefault;
import frc.robot.subsystems.utils.MotionProfileClimberDouble;


/**
 * Add your docs here.
 */
public class MotionProfileClimber extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  TalonSRX _talon1 = null;
  TalonSRX _talon2 = null;
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

  /** some example logic on how one can manage an MP */
  MotionProfileClimberDouble _example = null;

  private PodPosition face = PodPosition.FRONT;
  private PodPosition side = PodPosition.LEFT;

  //point of view of the face that we climb with
  public static enum PodPosition {
    FRONT, //collector side
    BACK,  //beak side
    LEFT,  //where front is the collector
    RIGHT  //where front is the collector
  }

  //point of view of the robot, not the pod mechanisms
  //e.g.:  when the robot belly pan rises, thats up
  public static enum ClimberDirection {
    UP,
    DOWN
  }
  
  public MotionProfileClimber(int left, 
                              int right, 
                              int dioIdTop,
                              int dioIdBottom,
                              int dioIdWall, 
                              PodPosition face,
                              PodPosition side) {
    this.face = face;
    this.side = side;
    _talon1 = new TalonSRX(left);
    _example = new MotionProfileClimberDouble(_talon1, null);
    switchTopF = new DigitalInput(dioIdTop);
    switchBottomF = new DigitalInput(dioIdBottom);
    switchWallF = new DigitalInput(dioIdWall);

    setupTalon();



  }

  public void invertTalon1(boolean invert) {
    _talon1.setInverted(invert);
  }

  public void invertTalon2(boolean invert) {
    _talon2.setInverted(invert);
  }

  public void setupTalon() {
		/* Factory Default all hardware to prevent unexpected behaviour */
		_talon1.configFactoryDefault();
		if (_talon2 != null) {
		  _talon2.configFactoryDefault();
		_talon2.set(ControlMode.Follower, _talon1.getDeviceID());
		}
		_talon1.clearMotionProfileTrajectories(); //online

		_talon1.changeMotionControlFramePeriod(5);
		if (_talon2 != null) {
		_talon2.changeMotionControlFramePeriod(5);
    }
    
    /**
     * the commands will call set inverted based on direction
     *
    if(side == PodPosition.RIGHT) {
      _talon1.setInverted(false);
      if (_talon2 != null) {
        _talon2.setInverted(true);
      }
    } else {
      _talon1.setInverted(true);
      if (_talon2 != null) {
        _talon2.setInverted(false);
      }

    }
    */

       
      

	
		/* Configure Selected Sensor for Motion Profile */
	  
			_talon1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,
												frc.robot.subsystems.utils.Constants.kPIDLoopIdx,
												Constants.kTimeoutMs);
			/* Keep sensor and motor in phase, postive sensor values when MC LEDs are green */
		_talon1.setSensorPhase(false);
		//_talon.setSelectedSensorPosition(0);
			
			/**
			 * Configure MotorController Neutral Deadband, disable Motor Controller when
			 * requested Motor Output is too low to process
			 */
			_talon1.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);
	
			/* Configure PID Gains, to be used with Motion Profile */
			_talon1.config_kF(Constants.kPIDLoopIdx, Constants.kGains.kF, Constants.kTimeoutMs);
			_talon1.config_kP(Constants.kPIDLoopIdx, Constants.kGains.kP, Constants.kTimeoutMs);
			_talon1.config_kI(Constants.kPIDLoopIdx, Constants.kGains.kI, Constants.kTimeoutMs);
			_talon1.config_kD(Constants.kPIDLoopIdx, Constants.kGains.kD, Constants.kTimeoutMs);
	
			/* Our profile uses 10ms timing */
			_talon1.configMotionProfileTrajectoryPeriod(10, Constants.kTimeoutMs); 
			
			/* Status 10 provides the trajectory target for motion profile AND motion magic */
			_talon1.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);
	  
		
	  }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new ClimberDefault(this));
  }

  public boolean limitTop() {
    if (switchTopF.get()) {
      return SWITCH_OPEN;
    } else {
      return SWITCH_CLOSED;
    }
  }

  public boolean limitBottom() {
    if (switchBottomF.get()) {
      return SWITCH_OPEN;
    } else {
      return SWITCH_CLOSED;
    }
  }

  public boolean limitWall() {
    if (switchWallF.get()) {
      return SWITCH_OPEN;
    } else {
      return SWITCH_CLOSED;
    }
  }

  public void resetEncoderPosition(int position) {
    _talon1.setSelectedSensorPosition(0);
  }

  public double getEncPosition() {
    return _talon1.getSelectedSensorPosition();
  }

  public void set(double output) {
    _talon1.set(ControlMode.PercentOutput, output);
  }

  public boolean isMotionProfileFinished() {
    return _example.isMotionProfileDone();
  }

  public MotionProfileClimberDouble getMP() {
    return _example;
  }

  public PodPosition getFace() {
    return face;
  }

  public PodPosition getSide() {
    return side;
  }

  public void setDirection(ClimberDirection direction) {
    if (side == PodPosition.LEFT) {
      if (direction == ClimberDirection.UP) {
        _talon1.setInverted(true);
      } else {
        _talon1.setInverted(false);
      }
    } else {
      if (direction == ClimberDirection.UP) {
        _talon1.setInverted(false);
      } else {
        _talon1.setInverted(true);
      }
    }
  }
}
