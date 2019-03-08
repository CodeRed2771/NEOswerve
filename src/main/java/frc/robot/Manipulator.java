package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.libs.CurrentBreaker;

public class Manipulator { // Should be changed to Manipulator.
    private static Manipulator instance;
    private static TalonSRX manipulator;
    private static CurrentBreaker currentBreaker;
    private static TalonSRX linkage;
    private static DoubleSolenoid flipper; // I know this is a dumb name. Sorry :)
    private static LimitSwitchNormal limitSwitch;

    private static boolean holdingCargo = false;
    private static boolean holdingHatch = false; // I kept it different in case we want to use this for logic regarding
                                                 // what to score.
    private static boolean intakeRunning = false;
    private static double ejectEndTime;

    public static Manipulator getInstance() {
        if (instance == null)
            instance = new Manipulator();
        return instance;
    }

    public Manipulator() {
        manipulator = new TalonSRX(Wiring.MANIPULATOR_MOTOR);
        linkage = new TalonSRX(Wiring.LINKAGE_MOTOR);

        manipulator.setInverted(true);

        manipulator.configOpenloopRamp(.2, 0);

        manipulator.setNeutralMode(NeutralMode.Brake);

        flipper = new DoubleSolenoid(Wiring.FLIPPER_PCM_PORTA, Wiring.FLIPPER_PCM_PORTB);

        currentBreaker = new CurrentBreaker(null, Wiring.INTAKE_PDP_PORT, Calibration.INTAKE_MAX_CURRENT, 250, 2000);

        resetIntakeStallDetector();

        ejectEndTime = aDistantFutureTime();

        /* first choose the sensor */
		linkage.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);

		/* set the relevant frame periods to be at least as fast as periodic rate */
		linkage.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 0);
		linkage.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 0);

		/* set the peak and nominal outputs */
		linkage.configNominalOutputForward(0, 0);
		linkage.configNominalOutputReverse(0, 0);
		linkage.configPeakOutputForward(1, 0);
        linkage.configPeakOutputReverse(-1, 0);

        linkage.configContinuousCurrentLimit(25);

		linkage.setNeutralMode(NeutralMode.Brake);

		linkage.configClosedloopRamp(.25, 0);

		/* set closed loop gains in slot0 - see documentation */
		linkage.selectProfileSlot(0, 0);

		linkage.config_kP(0, Calibration.LINKAGE_P, 0);
		linkage.config_kI(0, Calibration.LINKAGE_I, 0);
		linkage.config_kD(0, Calibration.LINKAGE_D, 0);
		linkage.config_kF(0, Calibration.LINKAGE_F, 0);

		SmartDashboard.putNumber("Link Vel", Calibration.LINKAGE_VELOCITY);
		SmartDashboard.putNumber("Link Accel", Calibration.LINKAGE_ACCEL);
		SmartDashboard.putNumber("Link P", Calibration.LINKAGE_P);
		SmartDashboard.putNumber("Link I", Calibration.LINKAGE_I);
		SmartDashboard.putNumber("Link D", Calibration.LINKAGE_D);
		SmartDashboard.putNumber("Link F", Calibration.LINKAGE_F);

		SmartDashboard.putBoolean("Link TUNE", false);
    }

    public static void tick() {

        SmartDashboard.putNumber("Link Enc", linkage.getSensorCollection().getQuadraturePosition());
        SmartDashboard.putNumber("Link Err", linkage.getClosedLoopError());

        if (SmartDashboard.getBoolean("Link TUNE", false)) {
			linkage.configMotionCruiseVelocity((int) SmartDashboard.getNumber("Link Vel", 0), 0);
			linkage.configMotionAcceleration((int) SmartDashboard.getNumber("Link Accel", 0), 0);
			linkage.config_kF(0, SmartDashboard.getNumber("Link F", 1.0), 0);
			linkage.config_kP(0, SmartDashboard.getNumber("Link P", 1.0), 0);
			linkage.config_kI(0, SmartDashboard.getNumber("Link I", 0), 0);
			linkage.config_kD(0, SmartDashboard.getNumber("Link D", 0), 0);
		}

        if (intakeStalled() && !holdingCargo) {
            holdGamePiece();
        }

        // this turns off the intake a little while after starting an eject
        if (System.currentTimeMillis() > ejectEndTime) {
            stopIntake();
            ejectEndTime = aDistantFutureTime();
        }

        if (linkage.getControlMode() == ControlMode.MotionMagic)
			SmartDashboard.putNumber("Link Setpt", linkage.getClosedLoopTarget());
		else
			SmartDashboard.putNumber("Link Setpt", -1);
    }

    // CONTROL METHODS ------------------------------------------------
    public static void linkageMove(double speed) {
        linkage.set(ControlMode.PercentOutput, speed);
    }
    
    public static void linkageDown() {
        linkage.set(ControlMode.MotionMagic, -900);
    }

    public static void linkageUp(){
        linkage.set(ControlMode.MotionMagic, 0);
    }

    public static void intakeCargo() {
        manipulator.set(ControlMode.PercentOutput, -1);

        intakeRunning = true;
        holdingCargo = false;
        resetIntakeStallDetector();
        ejectEndTime = aDistantFutureTime();
    }

    public static void intakeHatch() {
        manipulator.set(ControlMode.PercentOutput, .1);

        intakeRunning = true;
        holdingHatch = false;
        holdingCargo = false;
        resetIntakeStallDetector();
        ejectEndTime = aDistantFutureTime();
    }

    public static void lowerFlipper() {
        flipper.set(Value.kForward);
    }

    public static void bringFlipperUp() {
        flipper.set(Value.kReverse);
    }

    public static boolean isIntakeRunning() {
        return intakeRunning;
    }

    public static void holdGamePiece() {
        stopIntake();
        manipulator.set(ControlMode.PercentOutput, -.25);
        holdingCargo = true;
    }

    public static void ejectGamePiece() {
        if (holdingCargo) {
            manipulator.set(ControlMode.PercentOutput, 1);

            holdingCargo = false;
            resetIntakeStallDetector();

            ejectEndTime = System.currentTimeMillis() + 750;
        } else if (holdingHatch) {
            manipulator.set(ControlMode.PercentOutput, -1);

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
