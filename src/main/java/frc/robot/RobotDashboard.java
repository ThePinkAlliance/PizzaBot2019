/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * RobotDashboard: helper class to encapsulate displaying data to the dashboard.
 * This class has a boolean that can be set to false to stop displaying data.
 * 
 * The class instance is created after all subsystems and m_oi is created in Robot.java.
 * This ensures that we can access the subsystems to get their data and put it onto the
 * SmartDashboard.
 */
public class RobotDashboard {
    public static final String ENC_LABEL_RIGHT_FRONT =  "Right Front Distance(in)  ";
    public static final String ENC_LABEL_LEFT_FRONT  =  "Left Front Distance(in)   ";
    public static final String ENC_LABEL_FRONT       =  "Front Distance Average(in)";
    
    private boolean bDisplayContinuousData = true;

    /**
     * Constructor:  put values onto dashboard for the first time.
     */
    public RobotDashboard() {
       
       displayEncoderValues();
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
    }

    /**
     * Called from Robot.java during periodic states (teleop / autonomous)
     */
    public void displayContinuousData() {
        displayEncoderValues();
    }
}
