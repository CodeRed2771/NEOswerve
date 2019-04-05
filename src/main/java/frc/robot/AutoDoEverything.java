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
public class AutoDoEverything extends AutoBaseClass {

    private double distanceToTarget = 0;
    private double angleDiff;
    private double distToStayBackOnFirstDrive = 20;
    private double targetAngle = 0;
    private boolean drivingAllowed = true;

    public enum ActionMode {
        JUST_DRIVE, GET_HATCH, GET_CARGO, PLACE_HATCH, PLACE_CARGO;
    }

    private ActionMode actionMode = ActionMode.JUST_DRIVE;

    public void start() {
        super.start();
        Vision.setTargetTrackingMode();
    }

    public void stop() {
        super.stop();
        Vision.setDriverMode();
    }

    public void setActionMode(ActionMode actionMode) {
        this.actionMode = actionMode;
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
                DriveAuto.turnDegrees(Vision.offsetFromTarget(), .35);
                setTimerAndAdvanceStep(1000);
                break;
            case 2:
                if (DriveAuto.turnCompleted()) {
                    advanceStep();
                }
                break;
            case 3:
                angleDiff = RobotGyro.getClosestTurn(targetAngle);
                // angleDiff = targetAngle - RobotGyro.getRelativeAngle();
                System.out.println("anglediff " + angleDiff);
                DriveAuto.turnDegrees(angleDiff, .45); // Square up with target
                setTimerAndAdvanceStep(1000);
                break;
            case 4:
                if (DriveAuto.turnCompleted()) {
                    advanceStep();
                }
                break;
            case 5:
                double slideDistance = -((Math.sin(Math.toRadians(angleDiff)) * distanceToTarget) + 1);
                SmartDashboard.putNumber("Slide Dist", slideDistance);
                driveInches(slideDistance, 90, .5, false);
                setTimerAndAdvanceStep(3000);
                break;
            case 6:
                if (DriveAuto.hasArrived()) {
                    advanceStep();
                }
                break;
            case 7:
                // keep scanning for a distance reading
                distanceToTarget = Vision.getDistanceFromTarget();
                if (distanceToTarget > 0) {
                    advanceStep();
                }
                break;
            case 8:
                driveInches(distanceToTarget - distToStayBackOnFirstDrive, 0, .5, true);
                setTimerAndAdvanceStep(3000);
                break;
            case 9:
                if (DriveAuto.hasArrived()) {
                    advanceStep();
                }
                break;
            case 10:
                setStep(12);
                break;
            // case 10:
            // if (drivingAllowed) {
            // driveInches(distToStayBackOnFirstDrive, 0, .2, false);
            // setTimerAndAdvanceStep(1000);
            // } else {
            // setTimerAndAdvanceStep(20);
            // }
            // break;
            // case 11:
            // if (DriveAuto.hasArrived()) {
            // advanceStep();
            // }
            // break;
            case 12:
                if (actionMode == ActionMode.JUST_DRIVE) {
                    setStep(500);
                } else if (actionMode == ActionMode.GET_HATCH) {
                    setStep(20);
                } else if (actionMode == ActionMode.GET_CARGO) {
                    setStep(40);
                } else if (actionMode == ActionMode.PLACE_HATCH) {
                    setStep(60);
                } else if (actionMode == ActionMode.PLACE_CARGO) {
                    setStep(90);
                } else {
                    setStep(500);
                }
                break;

            /////////////////////////////////////////////////////////////////////////
            // Get HATCH
            /////////////////////////////////////////////////////////////////////////
            case 20:
                Manipulator.prepareToGetHatchFromFeeder();
                Manipulator.resetIntakeStallDetector();
                driveInches(30, 0, .3, true);
                setTimerAndAdvanceStep(2000);
                break;
            case 21:
            
                // if (Manipulator.intakeStalled()) {
                //     advanceStep();
                // }
                break;
            case 22:
                DriveAuto.stopDriving();
                // if (Manipulator.intakeStalled()) {
                    Manipulator.intakeHatchOverride();
                    // Manipulator.resetIntakeStallDetector();
                    setStep(23);
                // } else {
                //     setStep(500);
                // }
                break;
            case 23:
                DriveAuto.driveInches(-24, 0, .5);
                setTimerAndAdvanceStep(2000);
                break;
            case 24:
                if (DriveAuto.hasArrived()) {
                    advanceStep();
                }
                break;
            case 25:
                DriveAuto.stopDriving();
                setStep(500);
                break;

            /////////////////////////////////////////////////////////////////////////
            // GET CARGO
            /////////////////////////////////////////////////////////////////////////
            case 40:
                Manipulator.intakeCargoFeeder();
                driveInches(30, 0, .2);
                setTimerAndAdvanceStep(3000);
                break;
            case 41:
                if (Manipulator.intakeStalled()) {
                    advanceStep();
                }
                break;
            case 42:
                DriveAuto.stopDriving();
                driveInches(-2, 0, .3);
                break;
            case 43:
                if (DriveAuto.hasArrived()) {
                    advanceStep();
                }
                break;
            case 44:
                Manipulator.intakeCargoFeeder();
                setTimerAndAdvanceStep(5000);
                break;

            case 45:
                if (Manipulator.intakeStalled()) {
                    Manipulator.resetIntakeStallDetector();
                    setStep(46);
                } else {
                    setStep(500);
                }
                break;
            case 46:
                DriveAuto.driveInches(-24, 0, .5);
                setTimerAndAdvanceStep(2000);
                break;
            case 47:
                if (DriveAuto.hasArrived()) {
                    advanceStep();
                }
                break;
            case 48:
                DriveAuto.stopDriving();
                setStep(500);
                break;

            /////////////////////////////////////////////////////////////////////////
            // PLACE HATCH
            /////////////////////////////////////////////////////////////////////////
            case 60:
                Lift.goHatchLvl1();
                driveInches(30, 0, .3);
                setTimerAndAdvanceStep(2000);
                break;
            case 61:
                if (DriveAuto.hasArrived()) {
                    advanceStep();
                }
                break;
            case 62:
                Manipulator.moveFingerDown();
                setTimerAndAdvanceStep(300);
                break;
            case 63:
                break;
            case 64:
                driveInches(-12, 0, .25);
                setTimerAndAdvanceStep(2000);
                break;
            case 65:
                if (DriveAuto.hasArrived()) {
                    advanceStep();
                }
                break;
            case 66:
                setStep(500);
                break;

            /////////////////////////////////////////////////////////////////////////
            // PLACE CARGO
            /////////////////////////////////////////////////////////////////////////
            
            //Need to make sure Lift is at the right height
            case 90: 
                Lift.goCargoLvl1();
                driveInches(30, 0, .3); 
                setTimerAndAdvanceStep(2000);
                break;
            case 91:
                if (DriveAuto.hasArrived()) {
                    advanceStep();
                }
                break;
            case 92:
                Manipulator.ejectGamePiece();
                setTimerAndAdvanceStep(1000);
                break;
            case 93:
                break;
            case 94:
                driveInches(-12, 0, .25);
                setTimerAndAdvanceStep(2000);
                break;
            case 95:
                if (DriveAuto.hasArrived()) {
                    advanceStep();
                }
                break;
            case 96:
                DriveAuto.stopDriving();
                setStep(500);
                break;

            /////////////////////////////////////////////////////////////////////////
            // THE END
            /////////////////////////////////////////////////////////////////////////
            case 500:
                stop();
                break;
            }
        }

        SmartDashboard.putBoolean("Hatch running", isRunning());
        SmartDashboard.putNumber("tx", Vision.tx());
    }
}