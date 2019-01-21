/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.utils;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class DriveTrainPIDSourceLeft implements PIDSource {

    private PIDSourceType pidSourceType = PIDSourceType.kDisplacement;

    @Override 
  public double pidGet() {
     //Get the amount left to target
     return Robot.m_driveTrain.getFrontRightDistance();
  }

  @Override
  public void setPIDSourceType(PIDSourceType pidSourceType) {
    this.pidSourceType = pidSourceType;
  }

  @Override
  public PIDSourceType getPIDSourceType() {
    return pidSourceType;
  }


}