package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.libs.CurrentBreaker;

public class Manipulator { // Should be changed to Manipulator.
    private static Manipulator instance;
    private static TalonSRX manipulator;
    private static CurrentBreaker currentBreaker;
    private static DoubleSolenoid flipper; // I know this is a dumb name. Sorry :)
    private static LimitSwitchNormal limitSwitch;

    private static boolean holdingCargo = false;
    private static boolean holdingHatch = false; // I kept it different in case we want to use this for logic regarding
                                                 // what to score.
    private static boolean intakeRunning = false;
    private static double ejectEndTime;
    private static double startReverseTime;
    private static boolean reverseAllowed = false;

    public static Manipulator getInstance() {
        if (instance == null)
            instance = new Manipulator();
        return instance;
    }

    public Manipulator() {
        manipulator = new TalonSRX(Wiring.MANIPULATOR_MOTOR);
        manipulator.setInverted(true);

        manipulator.configOpenloopRamp(.2, 0);

        manipulator.setNeutralMode(NeutralMode.Brake);

        flipper = new DoubleSolenoid(Wiring.FLIPPER_PCM_PORTA, Wiring.FLIPPER_PCM_PORTB);

        currentBreaker = new CurrentBreaker(null, Wiring.INTAKE_PDP_PORT, Calibration.INTAKE_MAX_CURRENT, 250, 2000);

        resetIntakeStallDetector();

        ejectEndTime = aDistantFutureTime();
        startReverseTime = aDistantFutureTime();

    }

    public static void tick() {
        if (intakeStalled() && !holdingCargo) {
            holdGamePiece();
        }

        // this turns off the intake a little while after starting an eject
        if (System.currentTimeMillis() > ejectEndTime) {
            stopIntake();
            ejectEndTime = aDistantFutureTime();
        }
    }

    // CONTROL METHODS ------------------------------------------------
    public static void move(double speed) {
        // limit speed
        if (speed < -.3)
            speed = -.3;
        if (speed > .3)
            speed = .3;

        manipulator.set(ControlMode.PercentOutput, speed);
    }

    public static void intakeCargo() {
        manipulator.set(ControlMode.PercentOutput, -.8);

        intakeRunning = true;
        holdingCargo = false;
        resetIntakeStallDetector();
        ejectEndTime = aDistantFutureTime();
    }

    public static void intakeHatch() {
        manipulator.set(ControlMode.PercentOutput, .8);

        intakeRunning = true;
        holdingHatch = false;
        holdingCargo = false;
        resetIntakeStallDetector();
        ejectEndTime = aDistantFutureTime();
    }

    public static void lowerPneumaticActuator() {
        flipper.set(Value.kForward);
    }

    public static void bringPneumaticActuatorUp() {
        flipper.set(Value.kReverse);
    }

    public static boolean isIntakeRunning() {
        return intakeRunning;
    }

    public static void holdGamePiece() {
        stopIntake();
        manipulator.set(ControlMode.PercentOutput, -.15);
        holdingCargo = true;
    }

    public static void ejectGamePiece() {
        if (holdingCargo) {
            manipulator.set(ControlMode.PercentOutput, .5);

            holdingCargo = false;
            resetIntakeStallDetector();

            ejectEndTime = System.currentTimeMillis() + 750;
        } else if (holdingHatch) {
            manipulator.set(ControlMode.PercentOutput, -.5);

            holdingHatch = false;
            resetIntakeStallDetector();

            ejectEndTime = System.currentTimeMillis() + 750;
        }
    }

    public static void stopIntake() {
        manipulator.set(ControlMode.PercentOutput, 0);
        resetIntakeStallDetector();
        intakeRunning = false;
    }

    // UTILITY METHODS ---------------------------------------------------------

    public static boolean intakeStalled() {
        return (currentBreaker.tripped());
    }

    public static void resetIntakeStallDetector() {
        currentBreaker.reset();
    }

    private static double aDistantFutureTime() {
        return System.currentTimeMillis() + 900000; // 15 minutes in the future
    }

    /*
     * TEST METHODS
     */

    public static void testIntakeCargo(double speed) {
        manipulator.set(ControlMode.PercentOutput, speed);
    }

    public static void testEjectCargo(double speed) {
        manipulator.set(ControlMode.PercentOutput, -speed);
    }

}
