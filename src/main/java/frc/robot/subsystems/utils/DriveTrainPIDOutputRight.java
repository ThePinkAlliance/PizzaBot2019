/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.utils;

import edu.wpi.first.wpilibj.PIDOutput;
import frc.robot.*;

/**
 * Right side pid output implementation.
 */
public class DriveTrainPIDOutputRight implements PIDOutput {

    public DriveTrainPIDOutputRight() {

    }

    @Override
    public void pidWrite(double output) {
       Robot.m_driveTrain.rightMotor(output);
    }
}
