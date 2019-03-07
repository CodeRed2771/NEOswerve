package frc.robot;

import frc.robot.libs.HID.*;
import frc.robot.libs.HID.HID.Axis;

public class KeyMap {

    // GAMEPADS
    private final HID gp1 = new HID(0);
    private final HID gp2 = new HID(1);
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

    // Climbing
    private final HID.Button prepareToClimb = LogitechF310.DPAD_LEFT;
    private final HID.Button climb = LogitechF310.BUMPER_LEFT;
    private final HID.Axis manualClimberAxis = LogitechF310.STICK_LEFT_Y;

    // CONTROLLER 2
    private final HID.Button startIntake = LogitechF310.BUMPER_RIGHT;
    private final HID.Button ejectCargo = LogitechF310.BUMPER_LEFT;

    // Hatch Placement
    private final HID.Button hatchRocketLvl1 = LogitechF310.A;
    private final HID.Button hatchRocketLvl2 = LogitechF310.B;
    private final HID.Button bringLiftToStart = LogitechF310.X;
    private final HID.Button hatchRocketLvl3 = LogitechF310.Y;

    // Cargo Placement
    private final Axis modifier = LogitechF310.TRIGGER_RIGHT_AXIS;

    private final HID.Button manualLiftUp = LogitechF310.DPAD_UP;
    private final HID.Button manualLiftDown = LogitechF310.DPAD_DOWN;

    // TEST CONTROLLER
    private final HID.Button singleClimbRevolutionButton = LogitechF310.A;
    private final HID.Button singleClimbRevolutionReverseButton = LogitechF310.B;
    private final HID.Axis manualManipulator = LogitechF310.STICK_RIGHT_Y;

    public HID getHID(int gamepad) {
        if (!singleControllerMode) {
            switch (gamepad) {
            case gamepad1:
                return gp1;
            case gamepad2:
                return gp2;
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

    public boolean getSwitchToRobotCentric() {
        return getHID(gamepad1).button(switchToRobotCentric);
    }

    public double getSwerveYAxis() {
        return getHID(gamepad1).axis(swerveYAxis);
    }

    public double getSwerveRotAxis() {
        return getHID(gamepad1).axis(swerveRotAxis);
    }

    public boolean activateIntake() {
        return getHID(gamepad2).button(startIntake);
    }

    public boolean ejectCargo() {
        return getHID(gamepad2).button(ejectCargo);
    }

    public boolean getHatchRocketLvl1() {
        return getHID(gamepad2).button(hatchRocketLvl1);
    }

    public boolean getHatchRocketLvl2() {
        return getHID(gamepad2).button(hatchRocketLvl2);
    }

    public boolean getHatchRocketLvl3() {
        return getHID(gamepad2).button(hatchRocketLvl3);
    }

    public boolean getCargoRocketLvl1() {
        return getHID(gamepad2).button(hatchRocketLvl1) && getHID(gamepad2).axis(modifier) > 0.8;
    }

    public boolean getCargoRocketLvl2() {
        return getHID(gamepad2).button(hatchRocketLvl2) && getHID(gamepad2).axis(modifier) > 0.8;
    }

    public boolean getCargoRocketLvl3() {
        return getHID(gamepad2).button(hatchRocketLvl3) && getHID(gamepad2).axis(modifier) > 0.8;
    }

    public boolean getBringLiftToStart() {
        return getHID(gamepad2).button(bringLiftToStart);
    }

    public boolean getcargoShipPlacement() {
        return getHID(gamepad2).button(bringLiftToStart) && getHID(gamepad2).axis(modifier) > 0.8;
    }

    public boolean getManualLiftUp() {
        // return getHID(gamePadNumber).button(LogitechF310.DPAD_UP);
        return getHID(gamepad2).button(manualLiftUp);
    }

    public boolean getManualLiftDown() {
        return getHID(gamepad2).button(manualLiftDown);
    }

    public boolean getPrepareToClimb() {
        return getHID(gamepad1).button(prepareToClimb) && getHID(gamepad1).axis(modifier) > 0.8;
    }

    public boolean getClimb() {
        return getHID(gamepad1).button(climb);
    }

    public double getManualClimb() {
        return getHID(gamepad1).axis(manualClimberAxis);
    }

    public boolean getSingleClimbRevolution() {
        return getHID(gamepad3).button(singleClimbRevolutionButton);
    }

    public boolean getSingleClimbReverseRevolution() {
        return getHID(gamepad3).button(singleClimbRevolutionReverseButton);
    }

    public double getManualManipulator() {
        return getHID(gamepad3).axis(manualManipulator);
    }

    /*
     * 
     * Provide some direct access to the commonly used F310 gamepad features
     * 
     */
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

}
