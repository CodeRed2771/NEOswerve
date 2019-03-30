package frc.robot;

import frc.robot.libs.HID.*;
import frc.robot.libs.HID.HID.Axis;

public class KeyMap {

    // GAMEPADS
    private final HID gp1 = new HID(0);
    private final HID gp2 = new HID(1);
    private final HID gp3 = new HID(2);
    private final int gamepad1 = 0;
    private final int gamepad2 = 1;
    private final int gamepad3 = 2;

    // MANAGEMENT BOOLEANS
    private boolean singleControllerMode = false;

    // CONTROLLER 1
    private final HID.Axis swerveXAxis = LogitechF310.STICK_LEFT_X;
    private final HID.Axis swerveYAxis = LogitechF310.STICK_LEFT_Y;
    private final HID.Axis swerveRotAxis = LogitechF310.STICK_RIGHT_X;
    // private final HID.Axis swerveXAxis = LogitechExtreme3D.STICK_X;
    // private final HID.Axis swerveYAxis = LogitechExtreme3D.STICK_Y;
    // private final HID.Axis swerveRotAxis = LogitechExtreme3D.STICK_ROT;
    private final HID.Button switchToRobotCentric = LogitechF310.BUMPER_LEFT;

    private final HID.Button zeroGyro = LogitechF310.START;

    // private final HID.Button driveOffPlatform = LogitechF310.BUMPER_LEFT;

    // Climbing
    private final HID.Button climb = LogitechF310.DPAD_UP;
    private final HID.Button driveModifier = LogitechF310.BUMPER_RIGHT;
    private final HID.Button undoClimb = LogitechF310.DPAD_DOWN;
    
    // Auto Programs
    private final HID.Button shipMoveLeft = LogitechF310.DPAD_LEFT;
    private final HID.Button shipMoveRight = LogitechF310.DPAD_RIGHT;
    private final HID.Button findRocketTarget = LogitechF310.A;
    private final HID.Button findShipTarget = LogitechF310.B;
    private final HID.Button findFeedStation = LogitechF310.Y;

    // CONTROLLER 2
    private final HID.Button intakeCargo = LogitechF310.DPAD_LEFT;
    private final HID.Button intakeHatch = LogitechF310.DPAD_UP;
    private final HID.Button stopIntake = LogitechF310.DPAD_RIGHT;
    private final HID.Button gamePieceOverride = LogitechF310.DPAD_DOWN;
    private final HID.Axis ejectGamePiece = LogitechF310.TRIGGER_RIGHT_AXIS;
    private final HID.Button fingerUp = LogitechF310.START;

    // Hatch Placement
    private final HID.Button goToLvl1 = LogitechF310.A;
    private final HID.Button goToLvl2 = LogitechF310.B;
    private final HID.Button goToLvl3 = LogitechF310.Y;
    private final HID.Button goToShipCargo = LogitechF310.BUMPER_RIGHT;
    private final HID.Button goToTravelPosition = LogitechF310.X;
    private final HID.Button modifier = LogitechF310.BUMPER_LEFT;

    private final Axis manualLift = LogitechF310.STICK_LEFT_Y;
    private final HID.Axis manualClimbDrive = LogitechF310.STICK_RIGHT_Y; // Not used in robot Java
    

    // TEST CONTROLLER
    private final HID.Button singleClimbRevolutionButton = LogitechF310.A;
    private final HID.Button singleClimbRevolutionReverseButton = LogitechF310.B;
    private final HID.Axis manualClimb = LogitechF310.STICK_LEFT_Y;

    public HID getHID(int gamepad) {
        if (!singleControllerMode) {
            switch (gamepad) {
            case gamepad1:
                return gp1;
            case gamepad2:
                return gp2;
            case gamepad3:
                return gp3;
            default:
                return null;
            }
        } else {
            return gp1;
        }
    }

    public double getSwerveXAxis() {
        return getHID(gamepad1).axis(swerveXAxis);
    }

    public double getSwerveYAxis() {
        return getHID(gamepad1).axis(swerveYAxis);
    }

    public boolean getRobotCentricModifier() {
        return getHID(gamepad1).button(switchToRobotCentric);
    }

    public boolean getZeroGyro() {
        return getHID(gamepad1).button(zeroGyro);
    }

    public double getSwerveRotAxis() {
        return getHID(gamepad1).axis(swerveRotAxis);
    }

    public boolean activateCargoIntake() {
        return getHID(gamepad2).button(intakeCargo);
    }

    public boolean activateCargoFeedIntake() {
        return getHID(gamepad2).button(intakeCargo) && getHID(gamepad2).button(modifier);
    }

    public boolean activateHatchIntake() {
        return getHID(gamepad2).button(intakeHatch);
    }

    // public boolean activateHatchIntakeAuto() {
    //     return getHID(gamepad2).button(intakeHatch) && getHID(gamepad2).button(modifier);
    // }

    public boolean linkageUp() {
        return getHID(gamepad2).button(goToTravelPosition) && getHID(gamepad2).button(modifier);
    }

    public boolean turnIntakeOff() {
        return getHID(gamepad2).button(stopIntake);
    }

    public boolean gamePieceOverride() {
        return getHID(gamepad2).button(gamePieceOverride);
    }

    public boolean ejectGamePiece() {
        return getHID(gamepad2).axis(ejectGamePiece) > 0.8;
    }

    public boolean goToLvl1() {
        return getHID(gamepad2).button(goToLvl1);
    }

    public boolean goToLvl2() {
        return getHID(gamepad2).button(goToLvl2);
    }

    public boolean goToLvl3() {
        return getHID(gamepad2).button(goToLvl3);
    }

    public boolean getCargoShipPlacement() {
        return getHID(gamepad2).button(goToShipCargo);
    }

    public double getManualLift () {
        return getHID(gamepad2).axis(manualLift);
    }
    public boolean goToTravelPosition() {
        return getHID(gamepad1).button(goToTravelPosition) || getHID(gamepad2).button(goToTravelPosition);
    }

    public boolean findRocketTarget() {
        return getHID(gamepad1).button(findRocketTarget); 
    }

    public boolean findShipTarget() {
        return getHID(gamepad1).button(findShipTarget);
    }

    public boolean findFeedStation() {
        return getHID(gamepad1).button(findFeedStation);
    }

    public boolean shipMoveLeft() {
        return getHID(gamepad1).button(shipMoveLeft) && !getHID(gamepad1).button(driveModifier);
    }

    public boolean shipMoveRight() {
        return getHID(gamepad1).button(shipMoveRight) && !getHID(gamepad1).button(driveModifier);
    }
    
    public boolean getClimb() {
        return getHID(gamepad1).button(driveModifier) && getHID(gamepad1).button(climb);
    }

    public boolean getUndoClimb() {
        return getHID(gamepad1).button(driveModifier) && getHID(gamepad1).button(undoClimb);
    }

    public boolean getSingleClimbRevolution() {
        return getHID(gamepad3).button(singleClimbRevolutionButton);
    }

    public boolean getSingleClimbReverseRevolution() {
        return getHID(gamepad3).button(singleClimbRevolutionReverseButton);
    }

    public double getClimbDrive() {
        return getHID(gamepad2).axis(manualClimbDrive);
    }

    public boolean getFingerUp() {
        return getHID(gamepad2).button(fingerUp);
    }

    public boolean getHatchOverride() {
        return getHID(gamepad2).button(intakeHatch) && getHID(gamepad2).button(modifier);
    }

    public boolean getDriveModifier() {
        return getHID(gamepad1).button(driveModifier);
    }

    // public boolean driveOffPlatform() {
    //     return getHID(gamepad1).button(driveOffPlatform);
    // }

    //TEST METHODS
    public double manualClimb() {
        return getHID(gamepad3).axis(manualClimb);
    }


     /*
     * 
     * Provide some direct access to the commonly used F310 gamepad features
     * 
     */
    public boolean getBumperLeft(int gamePadNumber) {
        return getHID(gamePadNumber).button(LogitechF310.BUMPER_LEFT);
    }

    public boolean getButtonA(int gamePadNumber) {
        return getHID(gamePadNumber).button(LogitechF310.A);
    }

    public boolean getButtonB(int gamePadNumber) {
        return getHID(gamePadNumber).button(LogitechF310.B);
    }

    public boolean getButtonX(int gamePadNumber) {
        return getHID(gamePadNumber).button(LogitechF310.X);
    }

    public boolean getButtonY(int gamePadNumber) {
        return getHID(gamePadNumber).button(LogitechF310.Y);
    }

    public boolean getStartButton(int gamePadNumber) {
        return getHID(gamePadNumber).button(LogitechF310.START);
    }

    public boolean getDpadLeft(int gamePadNumber) {
        return getHID(gamePadNumber).button(LogitechF310.DPAD_LEFT);
    }

    public boolean getDpadRight(int gamePadNumber) {
        return getHID(gamePadNumber).button(LogitechF310.DPAD_RIGHT);
    }

    public boolean getDpadUp(int gamePadNumber) {
        return getHID(gamePadNumber).button(LogitechF310.DPAD_UP);
    }

    public boolean getDpadDown(int gamePadNumber) {
        return getHID(gamePadNumber).button(LogitechF310.DPAD_DOWN);
    }

    public double getLeftStickY(int gamePadNumber) {
        return getHID(gamePadNumber).axis(LogitechF310.STICK_LEFT_Y);
    }

    public double getRightStickY(int gamePadNumber) {
        return getHID(gamePadNumber).axis(LogitechF310.STICK_RIGHT_Y);
    }

}
