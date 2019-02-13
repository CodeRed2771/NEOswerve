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
public class FindHatch extends AutoBaseClass {

    private double distanceToTarget = 0;
    private double angleDiff;
    double distToStayBack = 30;

    public void start() {
        super.start();
        Vision.setTargetTrackingMode();
    }

    public void stop() {
        super.stop();
        Vision.setDriverMode();
    }

    public void tick() {
        
        if (isRunning()) {
            
            DriveAuto.tick();
            SmartDashboard.putNumber("Hatch Step", getCurrentStep());

			switch(getCurrentStep()) {
                case 0:
                    // keep scanning for a distance reading
                    distanceToTarget = Vision.getDistanceFromTarget();
                    if (distanceToTarget > 0) {
                        advanceStep();
                    }
                    break;
                case 1:
                    DriveAuto.turnDegrees(Vision.offsetFromTarget(), .5);
                    setTimerAndAdvanceStep(1000);
                    break;
                case 2:
                    if (DriveAuto.turnCompleted()) {
                        advanceStep();
                    }
                    break;
                case 3:
                    angleDiff = TargetInfo.targetAngle() - RobotGyro.getAngle();
                    DriveAuto.turnDegrees(angleDiff, .5); // Square up?
                    setTimerAndAdvanceStep(5000);
                    break;
                case 4:
                    if (DriveAuto.turnCompleted()) {
                        advanceStep();
                    }
                    break;
                case 5:
                    double opposite = Math.sin(Math.toRadians(angleDiff)) * distanceToTarget;
                    double adjacent = Math.cos(Math.toRadians(angleDiff)) * distanceToTarget;

                    double newDist = Math.sqrt(Math.pow(opposite, 2) + Math.pow((adjacent - distToStayBack), 2));
                    double newAngle = -Math.atan(opposite / (adjacent - distToStayBack)); // calculates strafe angle

                    newAngle = (newAngle * 180) / Math.PI; // Converts angle to degrees from radians.

                    newAngle = newAngle - angleDiff;

                    SmartDashboard.putNumber("Drive dist", newDist);
                    SmartDashboard.putNumber("Target Angle", TargetInfo.targetAngle());
                    SmartDashboard.putNumber("New Angle", newAngle);
                    SmartDashboard.putNumber("Angle Diff", angleDiff);
                    // SmartDashboard.putNumber("opposite", opposite);
                    // SmartDashboard.putNumber("adjacent", adjacent);
                
                    // DriveAuto.reset();
                    DriveAuto.driveInches(newDist, newAngle, .4);

                    setTimerAndAdvanceStep(3000);

                    break;
                case 6:
                    if (DriveAuto.hasArrived()) {
                        advanceStep();
                    }
                    break;
                case 7:
                    stop();
                    break;
                }
            }
        	
        SmartDashboard.putBoolean("Hatch running", isRunning());
		SmartDashboard.putNumber("tx", Vision.tx());
    }
}
