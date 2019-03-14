/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class AutoGrabHatchFromFeeder extends AutoBaseClass {

       public void start() {
        super.start();
    }

    public void stop() {
        super.stop();
    }

    public void tick() {

        if (isRunning()) {

            DriveAuto.tick();
            SmartDashboard.putNumber("Hatch Step", getCurrentStep());

            switch (getCurrentStep()) {
            case 0: 
                Manipulator.resetIntakeStallDetector();
                driveInches(35, 0, .3);
                Manipulator.prepareToGetHatchFromFeeder();  
                setTimerAndAdvanceStep(2000);          
                break;
            case 1:
                if (Manipulator.intakeStalled()) {
                    advanceStep();
                    Manipulator.resetIntakeStallDetector();
                }
                break;
            case 2: 
                DriveAuto.stopDriving();
                Manipulator.intakeHatch();
                setTimerAndAdvanceStep(1000);
                break;
            case 3:
                if (Manipulator.isHoldingHatch()) {
                    advanceStep();
                }
                break;
            case 4:
                driveInches(-30, 0, .4);
                setTimerAndAdvanceStep(1000);
                break;
            case 5:
                if (driveCompleted()) {
                    advanceStep();
                }
                break;
            case 6:
                stop();
                break;
            }
        }

    }
}
