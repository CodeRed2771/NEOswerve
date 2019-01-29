/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

/**
 * PID Source for vision based distance (FWD) PID
 */
public class FWDVision implements PIDSource{


    public PIDSourceType getPIDSourceType() {
		return PIDSourceType.kDisplacement;
	}

	public void setPIDSourceType(PIDSourceType pType) {

	}

	public double pidGet() {

		return Vision.getDistanceFromTarget();
	}
}
