/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.PIDOutput;

/**
 * This class collects the PID output of the forward PID
 * and provides the result to the calling method (usually swerve)
 */
public class FWDOutput implements PIDOutput {
    double value = 0;

    public void pidWrite(double output) {
        value = output;
    }

    public double getValue() {
        return value;
    }
}
