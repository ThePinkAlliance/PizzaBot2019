/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.commands.JoystickDrive;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.kauailabs.navx.frc.AHRS;


/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public static final String MYNAME = "DriveTrain";

  public static final int ESC_FRONT_RIGHT = 3;//3; <<<<<< CHANGED to 1 while testing motion profiling
  public static final int[] ENC_DIO_FRONT_RIGHT = {4,5};

  public static final int ESC_FRONT_LEFT = 5; //4;  <<<<<< CHANGED to 5 while testing motion profiling
  public static final int[] ENC_DIO_FRONT_LEFT = {6,7};

  public static final int ESC_REAR_RIGHT = 2;
  public static final int[] ENC_DIO_REAR_RIGHT = {2,3};

  public static final int ESC_REAR_LEFT = 1;
  public static final int[] ENC_DIO_REAR_LEFT = {0,1};

  public static final boolean ENC_INVERT_COUNT_FALSE = false;
  public static final boolean ENC_INVERT_COUNT_TRUE = true;

  //Encodering distance per pulse: No gearing to account for:
  public static final double WHEEL_DIAMETER = 6.0;
  public static final double PULSE_PER_REVOLUTION = 250;  //Need to revisit this value!!
  public final double DISTANCE_PER_PULSE = (double)(Math.PI*WHEEL_DIAMETER)/PULSE_PER_REVOLUTION;


  private WPI_TalonSRX _rightFront = null;
  private WPI_TalonSRX _rightRear = null;
  private WPI_TalonSRX _leftFront = null;
  private WPI_TalonSRX _leftRear = null;
  //private Faults _faults_L = new Faults();
  //private Faults _faults_R = new Faults();
  private DifferentialDrive _diffDrive = null;
  private double _governor = 1.0;

  private Encoder _enc_leftRear = null;
  private Encoder _enc_leftFront = null;
  private Encoder _enc_rightRear = null;
  private Encoder _enc_rightFront = null;
  private double  _enc_Kp = 1.0;
  private double  _enc_Ki = 0.0;
  private double  _enc_Kd = 0.0;

  private AHRS   _ahrs    = null;
  private double _navx_Kp = 1.0;
  private double _navx_Ki = 0.0;
  private double _navx_Kd = 0.0;
  public static final double NAVX_BAD_VALUE = -989898.0;


  public DriveTrain() {

    //Motor setup
    //_rightFront = new WPI_TalonSRX(ESC_FRONT_RIGHT);
    //_rightRear = new WPI_TalonSRX(ESC_REAR_RIGHT);
    //_leftFront = new WPI_TalonSRX(ESC_FRONT_LEFT);
    //_leftRear = new WPI_TalonSRX(ESC_REAR_LEFT);
    //_rightFront.setInverted(false);
    //_leftFront.setInverted(true);
    //_rightRear.setInverted(false);
    //_leftRear.setInverted(true);
    //setNeutralMode(NeutralMode.Brake);
    
    //_faults_L = new Faults();
    //_faults_R = new Faults();
    //_rightRear.set(ControlMode.Follower, ESC_FRONT_RIGHT);
    //_leftRear.set(ControlMode.Follower, ESC_FRONT_LEFT);

   
    //Encoder setup
    _enc_leftFront = null; //new Encoder(ENC_DIO_FRONT_LEFT[0], ENC_DIO_FRONT_LEFT[1], ENC_INVERT_COUNT_FALSE, Encoder.EncodingType.k4X);
    _enc_leftRear = null; //new Encoder(ENC_DIO_REAR_LEFT[0], ENC_DIO_REAR_LEFT[1], ENC_INVERT_COUNT_FALSE, Encoder.EncodingType.k4X);
    _enc_rightFront = null; //new Encoder(ENC_DIO_FRONT_RIGHT[0], ENC_DIO_FRONT_RIGHT[1], ENC_INVERT_COUNT_FALSE, Encoder.EncodingType.k4X);
    _enc_rightRear = null; //new Encoder(ENC_DIO_REAR_RIGHT[0], ENC_DIO_REAR_RIGHT[1], ENC_INVERT_COUNT_FALSE, Encoder.EncodingType.k4X);
    
    SetupEncoder(_enc_leftFront, "LEFTFRONT", true);
    SetupEncoder(_enc_leftRear, "LEFTREAR" , true);
    SetupEncoder(_enc_rightFront, "RIGHTFRONT", false);
    SetupEncoder(_enc_rightRear,  "RIGHTREAR", false);


    //_diffDrive = new DifferentialDrive(_leftFront, _rightFront);
    //_diffDrive = new DifferentialDrive(_leftRear, _rightRear);
    //_diffDrive.setRightSideInverted(false);

    try {
      /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
      /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
      /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
      //Per the link above:  on pizzabot the NavX is connected directly to the roborio so
      //SPI is the best option.
      _ahrs = new AHRS(SPI.Port.kMXP); 
      resetGyro();
    } catch (RuntimeException ex ) {
         System.out.println(MYNAME + ": Error instantiating navX-MXP:  " + ex.getMessage());
    }
    

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    //setDefaultCommand(new JoystickDrive());
  }

  public void setGovernor(double percent) {
    this._governor = percent;
  }

  public double getGovernor() {
    return this._governor;
  }

  public void stopDriveTrain() {
    //_diffDrive.tankDrive(0,0);
  }

  public void resetEncoders() {
    if (_enc_leftFront != null)
      _enc_leftFront.reset();
    if (_enc_leftRear != null)
      _enc_leftRear.reset();
    if (_enc_rightFront != null)
      _enc_rightFront.reset();
    if (_enc_rightRear != null)
      _enc_rightRear.reset();
  }

  private void SetupEncoder(Encoder enc, String name, boolean reverseDirection) {
    
    if (enc == null)
       return;
    enc.setName(name);
    System.out.println(MYNAME + ": Encoder: " + enc.getName());
    enc.setMaxPeriod(.1);
    enc.setMinRate(10);
    System.out.println(MYNAME + ": Distance per Pulse: " + DISTANCE_PER_PULSE);
    enc.setDistancePerPulse(DISTANCE_PER_PULSE);
    enc.setReverseDirection(reverseDirection);
    enc.setSamplesToAverage(7);
  }

  public double getFrontRightDistance() {
    if (_enc_rightFront != null)
       return _enc_rightFront.getDistance();
    else
       return 0.0;
  }

  public double getFrontLeftDistance() {
    if (_enc_leftFront != null)
       return _enc_leftFront.getDistance();
    else
       return 0.0;
  }
  public double getFrontDistanceAverage() {
    double left = getFrontLeftDistance();
    double right = getFrontRightDistance();
    //Make this more resilient if needed (e.g. base it off of single encoder)
    //in case one of them is null
    return (double)((left + right) / 2.0);
  }

  /**
   * Talon SRX specific: neutral modes are Coast or Brake
   * @param neutralMode
   */
  public void setNeutralMode(NeutralMode neutralMode) {
    if (_leftFront != null)
      _leftFront.setNeutralMode(neutralMode);
    if (_leftRear != null)
      _leftRear.setNeutralMode(neutralMode);
    if (_rightFront != null)
      _rightFront.setNeutralMode(neutralMode);
    if (_rightRear != null)
      _rightRear.setNeutralMode(neutralMode);
  }

  public void setEncKp(double value) {
    this._enc_Kp = value;
  }

  public void setEncKi(double value) {
    this._enc_Ki = value;
  }

  public void setEncKd(double value) {
    this._enc_Kd = value;
  }

  public double getEncKp() {
    return this._enc_Kp;
  }

  public double getEncKi() {
    return this._enc_Ki;
  }

  public double getEncKd() {
    return this._enc_Kd;
  }

  public void setNavxKp(double value) {
    this._navx_Kp = value;
  }

  public void setNavxKi(double value) {
    this._navx_Ki = value;
  }

  public void setNavxKd(double value) {
    this._navx_Kd = value;
  }

  public double getNavxKp() {
    return this._navx_Kp;
  }

  public double getNavxKi() {
    return this._navx_Ki;
  }

  public double getNavxKd() {
    return this._navx_Kd;
  }

  public double getGyroAngle() {
    if (_ahrs != null)
       return this._ahrs.getAngle();
    else
       return NAVX_BAD_VALUE;

  }

  public double getGyroYaw() {
    if (_ahrs != null)
       return this._ahrs.getYaw();
    else
       return NAVX_BAD_VALUE;
  }

  public void resetGyro() {
    if (_ahrs != null)
       this._ahrs.reset();
  }

  public AHRS getNavxPIDSource() {
    return this._ahrs;
  }

  public void rightMotor(double output) {
    if (_rightFront != null)
       _rightFront.set(output);
  }

  public void leftMotor(double output) {
    if (_leftFront != null)
    _leftFront.set(output);
  }

  public void rightMotorStop() {
    if (_rightFront != null)
    _rightFront.set(0);
  }

  public void leftMotorStop() {
    if (_leftFront != null)
    _leftFront.set(0);
  }

  public void tankDriveByEncoder(double left, double right) {
    
		//_diffDrive.tankDrive(left, right);
  }
  
  public void tankDriveByJoystick(double left, double right) {
    //System.out.println("Left: " + leftSpeed + " <===>  Right: " + rightSpeed);
    //For this setup (ESC forward green), If LEFT negative make positive, if positive make negative
    //For this setup (ESC forward green), If RIGHT positive make negative, if negative make positive
    left = left < 0 ? left*-1 : -left;
    right = right > 0 ? right*-1 : -right;
    //Apply governor for safety.  This brute safety needs to be taken into account
    //by either removing it or using it when tuning PID controllers, etc.
    //double leftGoverned = left * Math.abs(_governor);
    //double rightGoverned = right * Math.abs(_governor);
    //if (Math.abs(left) > 0.1  || Math.abs(right) > 0.1)
    //   System.out.println("Left: " + left +  " ---    Right: " + right);
    
		//_diffDrive.tankDrive(left, right);
	}
}
