/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DriveStraightByGyro;
import frc.robot.commands.EncoderBasedDrive;

/**
 * RobotDashboard: helper class to encapsulate displaying data to the dashboard.
 * This class has a boolean that can be set to false to stop displaying data.
 * 
 * The class instance is created after all subsystems and m_oi is created in Robot.java.
 * This ensures that we can access the subsystems to get their data and put it onto the
 * SmartDashboard.
 */
public class RobotDashboard {
    //Encoder data
    public static final String ENC_LABEL_RIGHT_FRONT =  "Right Front Distance(in)  ";
    public static final String ENC_LABEL_LEFT_FRONT  =  "Left Front Distance(in)   ";
    public static final String ENC_LABEL_FRONT       =  "Front Distance Average(in)";

    //PID for EncoderBasedDrive
    public static final String DT_ENC_PID_DISTANCE     = "DT_ENC_PID_DISTANCE";
    public static final String DT_ENC_PID_MAX_OUTPUT   = "DT_ENC_PID_OUTPUT";
    public static final String DT_ENC_Kp = "DT_ENC_Kp";
    public static final String DT_ENC_Ki = "DT_ENC_Ki";
    public static final String DT_ENC_Kd = "DT_ENC_Kd";

    //Gyro Data
    public static final String NAVX_LABEL_ANGLE = "NAVX Angle";
    public static final String NAVX_LABEL_YAW   = "NAVX Yaw";

    //PID for DriveStraightByGyro
    public static final String DT_NAVX_PID_ANGLE        = "DT_NAVX_PID_ANGLE";
    public static final String DT_NAVX_PID_MAX_OUTPUT   = "DT_NAVX_PID_OUTPUT";
    public static final String DT_NAVX_Kp = "DT_NAVX_Kp";
    public static final String DT_NAVX_Ki = "DT_NAVX_Ki";
    public static final String DT_NAVX_Kd = "DT_NAVX_Kd";

    
    private boolean bDisplayContinuousData = true;

    /**
     * Constructor:  put values onto dashboard for the first time.
     */
    public RobotDashboard() {
        //Handle initial values to seed the dashboard
        displayInitialValues();
    }

    /**
     * Can be called in Robot.java or anywhere else to turn off or on the calls
     * to put data on the dashboard.
     * @param bDisplay
     */
    public void setDisplayContinuousData(boolean bDisplay) {
        this.bDisplayContinuousData = bDisplay;
    }

    /**
     * Can be checked to see if we are putting data on the dashboard
     * @return bDisplayContinuousData
     */
    public boolean getDisplayContinuousData() {
        return this.bDisplayContinuousData;
    }

    /**
     * Prints any encoder related values to the dashboard
     */
    public void displayEncoderValues() {
        if (Robot.m_driveTrain != null) {
            SmartDashboard.putNumber(ENC_LABEL_RIGHT_FRONT, Robot.m_driveTrain.getFrontRightDistance());
            SmartDashboard.putNumber(ENC_LABEL_LEFT_FRONT, Robot.m_driveTrain.getFrontLeftDistance());
            SmartDashboard.putNumber(ENC_LABEL_FRONT, Robot.m_driveTrain.getFrontDistanceAverage());
           
        }
        if (Robot.m_climber2 != null) {
            SmartDashboard.putNumber("TALON 4 ENC ", Robot.m_climber2.getEncPosition());
        }
        if (Robot.m_climber != null) {
            SmartDashboard.putNumber("TALON 3 ENC ", Robot.m_climber.getEncPosition());
        }
    }

    public void displaySwitchValues() {
        if (Robot.m_climber != null) {
            SmartDashboard.putBoolean("CLIMBER_TOP_FRONT", Robot.m_climber.limitTop());
            SmartDashboard.putBoolean("CLIMBER_BOTTOMN_FRONT", Robot.m_climber.limitBottom());
            SmartDashboard.putBoolean("CLIMBER_WALL_FRONT", Robot.m_climber.limitWall());
        }
    }

    /**
     * Prints any gyro related values to the dashboard
     */
    public void displayGyroValues() {
        if (Robot.m_driveTrain != null) {
            SmartDashboard.putNumber(NAVX_LABEL_ANGLE, Robot.m_driveTrain.getGyroAngle());
            SmartDashboard.putNumber(NAVX_LABEL_YAW, Robot.m_driveTrain.getGyroYaw());
        }
    }

    /**
     * Display DriveTrain Encoder PID values
     */
    public void displayDriveTrainEncoderPIDValues() {
        //DriveTrain subsystem
        if (Robot.m_driveTrain != null) {
            SmartDashboard.putNumber(DT_ENC_PID_DISTANCE, EncoderBasedDrive.CMD_DEFAULT_DISTANCE);
            SmartDashboard.putNumber(DT_ENC_PID_MAX_OUTPUT, EncoderBasedDrive.CMD_MAX_OUTPUT);
            SmartDashboard.putNumber(DT_ENC_Kp, EncoderBasedDrive.CMD_Kp);
            SmartDashboard.putNumber(DT_ENC_Ki, EncoderBasedDrive.CMD_Ki);
            SmartDashboard.putNumber(DT_ENC_Kd, EncoderBasedDrive.CMD_Kd);
        }
    }

    /**
     * Display DriveTrain Navx PID values
     */
    public void displayDriveTrainNavxPIDValues() {
        //DriveTrain subsystem
        if (Robot.m_driveTrain != null) {
            SmartDashboard.putNumber(DT_NAVX_PID_ANGLE, DriveStraightByGyro.CMD_DEFAULT_ANGLE);
            SmartDashboard.putNumber(DT_NAVX_PID_MAX_OUTPUT, DriveStraightByGyro.CMD_MAX_OUTPUT);
            SmartDashboard.putNumber(DT_NAVX_Kp, DriveStraightByGyro.CMD_Kp);
            SmartDashboard.putNumber(DT_NAVX_Ki, DriveStraightByGyro.CMD_Ki);
            SmartDashboard.putNumber(DT_NAVX_Kd, DriveStraightByGyro.CMD_Kd);
        }
    }

    /**
     * Grab the DriveTrain Encoder PID values from the Dashboard and set them 
     * on the drive train
     */
    public void getDriveTrainEncoderPIDValues() {
        //DriveTrain subsystem
        if (Robot.m_driveTrain != null) {
            Robot.m_driveTrain.setEncKp(SmartDashboard.getNumber(DT_ENC_Kp, EncoderBasedDrive.CMD_Kp));
            Robot.m_driveTrain.setEncKi(SmartDashboard.getNumber(DT_ENC_Ki, EncoderBasedDrive.CMD_Ki));
            Robot.m_driveTrain.setEncKd(SmartDashboard.getNumber(DT_ENC_Kd, EncoderBasedDrive.CMD_Kd));
        }
    }

    /**
     * Grab the DriveTrain Navx PID values from the Dashboard and set them 
     * on the drive train
     */
    public void getDriveTrainNavxPIDValues() {
        //DriveTrain subsystem
        if (Robot.m_driveTrain != null) {
            Robot.m_driveTrain.setNavxKp(SmartDashboard.getNumber(DT_NAVX_Kp, DriveStraightByGyro.CMD_Kp));
            Robot.m_driveTrain.setNavxKi(SmartDashboard.getNumber(DT_NAVX_Ki, DriveStraightByGyro.CMD_Ki));
            Robot.m_driveTrain.setNavxKd(SmartDashboard.getNumber(DT_NAVX_Kd, DriveStraightByGyro.CMD_Kd));
        }
    }

    /**
     * Grabs values from the dashboard on a continuous basis (telelop / autonomous)
     */
    public void getContinuousData() {
        getDriveTrainEncoderPIDValues();
        getDriveTrainNavxPIDValues();
    }

    /**
     * Prints one time values to the dashboard to establish the widget
     */
    public void displayInitialValues() {
        displayDriveTrainEncoderPIDValues();
        displayDriveTrainNavxPIDValues();
        displayEncoderValues();
        displayGyroValues();
    }

    /**
     * Called from Robot.java during periodic states (teleop / autonomous)
     */
    public void displayContinuousData() {
        displayEncoderValues();
        displayGyroValues();
        displaySwitchValues();
    }
}
