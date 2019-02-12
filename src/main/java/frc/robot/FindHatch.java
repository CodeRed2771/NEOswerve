/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.libs.Timer;

/**
 * Add your docs here.
 */
public class FindHatch {
    private boolean isActive = false;
    private double distanceToTarget = 0;
    private Timer autoTimer = new Timer();

    public void start() {
        isActive = true;
        Vision.setTargetTrackingMode();
        autoTimer.setStage(0);
    }

    public void stop() {
        DriveAuto.stop();
        isActive = false;
        Vision.setDriverMode();
    }

    public boolean isRunning() {
        return isActive;
    }

    public void tick() {
        autoTimer.tick();
        
        if (isActive) {
            
            DriveAuto.tick();
            SmartDashboard.putNumber("Hatch Step", autoTimer.getStage());

			switch(autoTimer.getStage()) {
                case 0:
                    // keep scanning for a distance reading
                    distanceToTarget = Vision.getDistanceFromTarget();
                    if (distanceToTarget > 0) {
                        autoTimer.nextStage();
                    }
                    break;
                case 1:
                    DriveAuto.turnDegrees(Vision.offsetFromTarget(), .2);
                    autoTimer.setTimerAndAdvanceStage(1000);
                    break;
                case 2:
                    if (DriveAuto.turnCompleted()) {
                        autoTimer.nextStage();
                    }
                    break;
                case 3:
                    double distToStayBack = 48;
                    double angleDiff = TargetInfo.targetAngle() - RobotGyro.getAngle();
                    double opposite = Math.sin(Math.toRadians(angleDiff)) * distanceToTarget;
                    double adjacent = Math.cos(Math.toRadians(angleDiff)) * distanceToTarget;

                    double newDist = Math.sqrt(Math.pow(opposite, 2) + Math.pow((adjacent - distToStayBack), 2));
                    double newAngle = -Math.atan(opposite / (adjacent - distToStayBack));

                    newAngle = (newAngle * 180) / Math.PI;

                    SmartDashboard.putNumber("Drive dist", newDist);
                    SmartDashboard.putNumber("Target Angle", TargetInfo.targetAngle());
                    SmartDashboard.putNumber("New Angle", newAngle);
                    SmartDashboard.putNumber("Angle Diff", angleDiff);
                    // SmartDashboard.putNumber("opposite", opposite);
                    // SmartDashboard.putNumber("adjacent", adjacent);
                
                    // DriveAuto.reset();
                    DriveAuto.driveInches(newDist, newAngle, .4);

                    autoTimer.setTimerAndAdvanceStage(3000);

                    break;
                case 4:
                    if (DriveAuto.hasArrived()) {
                        autoTimer.nextStage();
                    }
                    break;
                case 5:
                    isActive = false;
                    break;
                }
            }
        	
        SmartDashboard.putBoolean("Hatch running", isActive);
		SmartDashboard.putNumber("tx", Vision.tx());
    }
}
