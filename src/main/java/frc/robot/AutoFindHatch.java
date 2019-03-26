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
public class AutoFindHatch extends AutoBaseClass {

    private double distanceToTarget = 0;
    private double angleDiff;
    private double distToStayBack = 30;
    private double targetAngle = 0;
    private boolean drivingAllowed = false;

    public void start() {
        super.start();
        Vision.setTargetTrackingMode();
    }

    public void stop() {
        super.stop();
        Vision.setDriverMode();
    }

    public void setDrivingAllowed(boolean isDrivingAllowed) {
        drivingAllowed = isDrivingAllowed;
    }

    public void tick() {

        if (isRunning()) {

            DriveAuto.tick();
            SmartDashboard.putNumber("Hatch Step", getCurrentStep());

            switch (getCurrentStep()) {
            case 0:
                // keep scanning for a distance reading
                distanceToTarget = Vision.getDistanceFromTarget();
                if (distanceToTarget > 0) {
                    advanceStep();
                }
                break;
            case 1:
                System.out.println("target type " + mTargetType.toString());
                targetAngle = TargetInfo.targetAngle(mTargetType);
                System.out.println("Target Angle " + targetAngle);
                DriveAuto.turnDegrees(Vision.offsetFromTarget(), .25);
                setTimerAndAdvanceStep(1000);
                break;
            case 2:
                if (DriveAuto.turnCompleted()) {
                    advanceStep();
                }
                break;
            case 3:
                angleDiff = targetAngle - RobotGyro.getAngle();
                System.out.println("anglediff " + angleDiff);
                DriveAuto.turnDegrees(angleDiff, .25); // Square up?
                setTimerAndAdvanceStep(1000);
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

                // newAngle = newAngle - angleDiff;

                SmartDashboard.putNumber("Dist to target", distanceToTarget);
                SmartDashboard.putNumber("Drive dist", newDist);
       
                SmartDashboard.putNumber("New Angle", newAngle);
                SmartDashboard.putNumber("Angle Diff", angleDiff);
                // SmartDashboard.putNumber("opposite", opposite);
                // SmartDashboard.putNumber("adjacent", adjacent);

                // DriveAuto.reset();
                if (drivingAllowed) {
                    DriveAuto.driveInches(newDist, newAngle, .4, true);

                    setTimerAndAdvanceStep(3000); 
                }
                 else {
                     setTimerAndAdvanceStep(20);
                 }
                break;
            case 6:
                if (DriveAuto.hasArrived()) {
                    advanceStep();
                }
                break;
            case 7:
                // keep scanning for a distance reading
                distanceToTarget = Vision.getDistanceFromTarget();
                angleDiff = Vision.offsetFromTarget();
                SmartDashboard.putNumber("Dist To Targ", distanceToTarget);
                SmartDashboard.putNumber("Angle Diff", angleDiff);
                if (distanceToTarget > 0) {
                    advanceStep();
                }
                break;
            case 8:
                double slideDistance = Math.sin(Math.toRadians(angleDiff)) * distanceToTarget + 1;
                SmartDashboard.putNumber("Slide Dist", slideDistance);
                driveInches(slideDistance, 90, .3);
                setTimerAndAdvanceStep(3000);
                break;
            case 9:
                if (DriveAuto.hasArrived()) {
                    advanceStep();
                }
                break;
            case 10:
                if (drivingAllowed) {
                    driveInches(distanceToTarget - 8, 0, .25, true);
                    setTimerAndAdvanceStep(1000);    
                } else {
                    setTimerAndAdvanceStep(20);
                }
                break;
            case 11:
                if (DriveAuto.hasArrived()) {
                    advanceStep();
                }
                break;
            case 12:
                stop();
                break;
            }
        }

        SmartDashboard.putBoolean("Hatch running", isRunning());
        SmartDashboard.putNumber("tx", Vision.tx());
    }
}
