/**
 * Example logic for firing and managing motion profiles.
 * This example sends MPs, waits for them to finish
 * Although this code uses a CANTalon, nowhere in this module do we changeMode() or call set() to change the output.
 * This is done in Robot.java to demonstrate how to change control modes on the fly.
 * 
 * The only routines we call on Talon are....
 * 
 * changeMotionControlFramePeriod
 * 
 * getMotionProfileStatus		
 * clearMotionProfileHasUnderrun     to get status and potentially clear the error flag.
 * 
 * pushMotionProfileTrajectory
 * clearMotionProfileTrajectories
 * processMotionProfileBuffer,   to push/clear, and process the trajectory points.
 * 
 * getControlMode, to check if we are in Motion Profile Control mode.
 * 
 * Example of advanced features not demonstrated here...
 * [1] Calling pushMotionProfileTrajectory() continuously while the Talon executes the motion profile, thereby keeping it going indefinitely.
 * [2] Instead of setting the sensor position to zero at the start of each MP, the program could offset the MP's position based on current position. 
 */
package frc.robot.subsystems.utils;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.Notifier;
import frc.robot.subsystems.MotionProfileClimber;

import com.ctre.phoenix.motion.*;

public class MotionProfileClimberDoubleInProgress {

	/**
	 * The status of the motion profile executer and buffer inside the Talon.
	 * Instead of creating a new one every time we call getMotionProfileStatus,
	 * keep one copy.
	 */
	private MotionProfileStatus _status = new MotionProfileStatus();

	/** additional cache for holding the active trajectory point */
	double _pos=0,_vel=0,_heading=0;

	/**
	 * reference to the talon we plan on manipulating. We will not changeMode()
	 * or call set(), just get motion profile status and make decisions based on
	 * motion profile.
	 */
	private TalonSRX _talon1;
	private TalonSRX _talon2;
	/**
	 * State machine to make sure we let enough of the motion profile stream to
	 * talon before we fire it.
	 */
	private int _state = 0;
	/**
	 * Any time you have a state machine that waits for external events, its a
	 * good idea to add a timeout. Set to -1 to disable. Set to nonzero to count
	 * down to '0' which will print an error message. Counting loops is not a
	 * very accurate method of tracking timeout, but this is just conservative
	 * timeout. Getting time-stamps would certainly work too, this is just
	 * simple (no need to worry about timer overflows).
	 */
	private int _loopTimeout = -1;
	/**
	 * If start() gets called, this flag is set and in the control() we will
	 * service it.
	 */
	private boolean _bStart = false;

	/**
	 * Since the CANTalon.set() routine is mode specific, deduce what we want
	 * the set value to be and let the calling module apply it whenever we
	 * decide to switch to MP mode.
	 */
	private SetValueMotionProfile _setValue = SetValueMotionProfile.Disable;
	/**
	 * How many trajectory points do we wait for before firing the motion
	 * profile.
	 */
	private static final int kMinPointsInTalon = 5;
	/**
	 * Just a state timeout to make sure we don't get stuck anywhere. Each loop
	 * is about 20ms.
	 */
	private static final int kNumLoopsTimeout = 10;
	
	/**
	 * Lets create a periodic task to funnel our trajectory points into our talon.
	 * It doesn't need to be very accurate, just needs to keep pace with the motion
	 * profiler executer.  Now if you're trajectory points are slow, there is no need
	 * to do this, just call _talon1.processMotionProfileBuffer() in your teleop loop.
	 * Generally speaking you want to call it at least twice as fast as the duration
	 * of your trajectory points.  So if they are firing every 20ms, you should call 
	 * every 10ms.
	 */
	class PeriodicRunnable1 implements java.lang.Runnable {
		public void run() {  
						//System.out.println("Runnable: " + _setValue);
						_talon1.processMotionProfileBuffer();    
					}
	}
	class PeriodicRunnable2 implements java.lang.Runnable {
		public void run() {  
						//System.out.println("Runnable: " + _setValue);
						_talon2.processMotionProfileBuffer();    
					}
	}
	
	
	Notifier _notifier1 = null;
	Notifier _notifier2 = null;

    private String _direction = MotionProfileClimber.DIRECTION_UP; 

	/**
	 * C'tor
	 * 
	 * @param talon
	 *            reference to Talon object to fetch motion profile status from.
	 */
	public MotionProfileClimberDoubleInProgress(TalonSRX talon1, TalonSRX talon2) {
		_talon1 = talon1;
		_talon2 = talon2;
		

		/*
		 * since our MP is 10ms per point, set the control frame rate and the
		 * notifer to half that
		 */
		_talon1.changeMotionControlFramePeriod(5);
		_talon2.changeMotionControlFramePeriod(5);
		_notifier1 = new Notifier(new PeriodicRunnable1());
		_notifier1.startPeriodic(0.005);
		_notifier2 = new Notifier(new PeriodicRunnable2());
		_notifier2.startPeriodic(0.005);
		//_notifier.stop();
	}

	/**
	 * Called to clear Motion profile buffer and reset state info during
	 * disabled and when Talon is not in MP control mode.
	 */
	public void reset() {
		/*
		 * Let's clear the buffer just in case user decided to disable in the
		 * middle of an MP, and now we have the second half of a profile just
		 * sitting in memory.
		 */
		_talon1.clearMotionProfileTrajectories();
		_talon2.clearMotionProfileTrajectories();
		/* When we do re-enter motionProfile control mode, stay disabled. */
		_setValue = SetValueMotionProfile.Disable;
		/* When we do start running our state machine start at the beginning. */
		_state = 0;
		_loopTimeout = -1;
		/*
		 * If application wanted to start an MP before, ignore and wait for next
		 * button press
		 */
		_bStart = false;
	}

	/**
	 * Called every loop.
	 */
	public void control(boolean movingUp) {
		/* Get the motion profile status every loop */
		_talon1.getMotionProfileStatus(_status);

		/*
		 * track time, this is rudimentary but that's okay, we just want to make
		 * sure things never get stuck.
		 */
		if (_loopTimeout < 0) {
			/* do nothing, timeout is disabled */
		} else {
			/* our timeout is nonzero */
			if (_loopTimeout == 0) {
				/*
				 * something is wrong. Talon is not present, unplugged, breaker
				 * tripped
				 */
				Instrumentation.OnNoProgress();
			} else {
				--_loopTimeout;
			}
		}

		/* first check if we are in MP mode */
		if (_talon1.getControlMode() != ControlMode.MotionProfile) {
			/*
			 * we are not in MP mode. We are probably driving the robot around
			 * using gamepads or some other mode.
			 */
			_state = 0;
			_loopTimeout = -1;
		} else {
			/*
			 * we are in MP control mode. That means: starting Mps, checking Mp
			 * progress, and possibly interrupting MPs if thats what you want to
			 * do.
			 */
			switch (_state) {
				case 0: /* wait for application to tell us to start an MP */
					if (_bStart) {
						_bStart = false;
	
						_setValue = SetValueMotionProfile.Disable;
						startFilling(movingUp);
						/*
						 * MP is being sent to CAN bus, wait a small amount of time
						 */
						_state = 1;
						_loopTimeout = kNumLoopsTimeout;
					}
					break;
				case 1: /*
						 * wait for MP to stream to Talon, really just the first few
						 * points
						 */
					/* do we have a minimum numberof points in Talon */
					if (_status.btmBufferCnt > kMinPointsInTalon) {
						/* start (once) the motion profile */
						_setValue = SetValueMotionProfile.Enable;
						/* MP will start once the control frame gets scheduled */
						_state = 2;
						_loopTimeout = kNumLoopsTimeout;
					}
					break;
				case 2: /* check the status of the MP */
					/*
					 * if talon is reporting things are good, keep adding to our
					 * timeout. Really this is so that you can unplug your talon in
					 * the middle of an MP and react to it.
					 */
					if (_status.isUnderrun == false) {
						_loopTimeout = kNumLoopsTimeout;
					}
					/*
					 * If we are executing an MP and the MP finished, start loading
					 * another. We will go into hold state so robot servo's
					 * position.
					 */
					if (_status.activePointValid && _status.isLast) {
						/*
						 * because we set the last point's isLast to true, we will
						 * get here when the MP is done
						 */
						_setValue = SetValueMotionProfile.Hold;
						_state = 0;
						_loopTimeout = -1;
					} else {
						//_talon1.set(ControlMode.MotionProfile, _setValue.value);
						_setValue = SetValueMotionProfile.Enable;
						_talon1.set(ControlMode.MotionProfile, _setValue.value);
						//_talon1.processMotionProfileBuffer();
					}
					break;
			}

			/* Get the motion profile status every loop */
			_talon1.getMotionProfileStatus(_status);
			//_heading = _talon1.getActiveTrajectoryHeading();
			_pos = _talon1.getActiveTrajectoryPosition();
			_vel = _talon1.getActiveTrajectoryVelocity();
			//_talon1.processMotionProfileBuffer();

			/* printfs and/or logging */
			//Instrumentation.process(_status, _pos, _vel, _heading);
		}
	}

	/** Start filling the MPs to all of the involved Talons. */
	private void startFilling(boolean movingUp) {
		/* since this example only has one talon, just update that one */
		if (movingUp == true)
		  startFilling(GeneratedClimberUp.Points, GeneratedClimberUp.kNumPoints);
		else
		  startFilling(GeneratedClimberDown.Points, GeneratedClimberDown.kNumPoints);
	}

	private void startFilling(double[][] profile, int totalCnt) {

		/* create an empty point */
		TrajectoryPoint point = new TrajectoryPoint();

		/* did we get an underrun condition since last time we checked ? */
		if (_status.hasUnderrun) {
			/* better log it so we know about it */
			Instrumentation.OnUnderrun();
			/*
			 * clear the error. This flag does not auto clear, this way 
			 * we never miss logging it.
			 */
			_talon1.clearMotionProfileHasUnderrun(0);
		}
		/*
		 * just in case we are interrupting another MP and there is still buffer
		 * points in memory, clear it.
		 */
		_talon1.clearMotionProfileTrajectories();

		/* set the base trajectory period to zero, use the individual trajectory period below */
		_talon1.configMotionProfileTrajectoryPeriod(Constants.kBaseTrajPeriodMs, Constants.kTimeoutMs);
		
		/* This is fast since it's just into our TOP buffer */
		for (int i = 0; i < totalCnt; ++i) {
			double positionRot = profile[i][0];
			double velocityRPM = profile[i][1];
			/* for each point, fill our structure and pass it to API */
			point.position = positionRot * Constants.kSensorUnitsPerRotation; //Convert Revolutions to Units
			point.velocity = velocityRPM * Constants.kSensorUnitsPerRotation / 600.0; //Convert RPM to Units/100ms
			point.headingDeg = 0; /* future feature - not used in this example*/
			point.profileSlotSelect0 = 0; /* which set of gains would you like to use [0,3]? */
			point.profileSlotSelect1 = 0; /* future feature  - not used in this example - cascaded PID [0,1], leave zero */
			point.timeDur = (int)profile[i][2];
			point.zeroPos = false;
			if (i == 0)
				point.zeroPos = true; /* set this to true on the first point */

			point.isLastPoint = false;
			if ((i + 1) == totalCnt)
				point.isLastPoint = true; /* set this to true on the last point  */

			_talon1.pushMotionProfileTrajectory(point);
		}
	}
	/**
	 * Called by application to signal Talon to start the buffered MP (when it's
	 * able to).
	 */
	public void startMotionProfile() {
		_bStart = true;
	}

	/**
	 * 
	 * @return the output value to pass to Talon's set() routine. 0 for disable
	 *         motion-profile output, 1 for enable motion-profile, 2 for hold
	 *         current motion profile trajectory point.
	 */
	public SetValueMotionProfile getSetValue() {
		return _setValue;
	}

	public boolean isMotionProfileDone() {
		if (_status.activePointValid &&  _status.isLast) {
			System.out.println("Last point reached!");
	        return true;
		}
		else
		   return false;
	}

	public void setupTalon(boolean invert) {
		/* Factory Default all hardware to prevent unexpected behaviour */
		_talon2.configFactoryDefault();
		if (_talon2 != null) {
		_talon2.configFactoryDefault();
		_talon2.setInverted(invert);
		_talon2.set(ControlMode.Follower, _talon1.getDeviceID());
		}
		_talon1.clearMotionProfileTrajectories(); //online

		_talon1.changeMotionControlFramePeriod(5);
		if (_talon2 != null) {
		_talon2.changeMotionControlFramePeriod(5);
		}
		_talon1.setInverted(invert);

	
		/* Configure Selected Sensor for Motion Profile */
	  
			_talon1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,
												Constants.kPIDLoopIdx,
												Constants.kTimeoutMs);
			/* Keep sensor and motor in phase, postive sensor values when MC LEDs are green */
		_talon1.setSensorPhase(true);
		//_talon1.setSelectedSensorPosition(0);
			
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

	  public void stopMotionProfile() {
		_talon1.set(ControlMode.MotionProfile, SetValueMotionProfile.Hold.value);
		_talon1.clearMotionProfileTrajectories();
		_talon1.set(ControlMode.MotionProfile, SetValueMotionProfile.Disable.value);
		_talon1.set(ControlMode.PercentOutput, 0);
		_setValue = SetValueMotionProfile.Disable;
		//reset();
	  }
	
	  //public void resetMotionProfile() {
		//reset();
		
	  //}
	
	  public void setMotionProfileMode() {
		SetValueMotionProfile setOutput = getSetValue();
		   _talon1.set(ControlMode.MotionProfile, setOutput.value);
	  }
	
	  
}
