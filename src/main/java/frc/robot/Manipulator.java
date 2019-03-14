package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
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
    private static DigitalInput limitSwitch;

    private static boolean linkageEncoderSet = false;
	private static boolean linkageEncoderSetting = false;
	private static double linkageEncoderSettingStartTime = System.currentTimeMillis();

    private enum ManipulatorState {
        INACTIVE,
        GETTING_CARGO,
        GETTING_HATCH,
        GETTING_HATCH_FLOOR,
        HOLDING_CARGO,
        HOLDING_HATCH,
        HOLDING_HATCH_FLOOR,
    }

    private static ManipulatorState manipulatorState;
    private static ManipulatorState previousState;

    private static double ejectEndTime;

    public static Manipulator getInstance() {
        if (instance == null)
            instance = new Manipulator();
        return instance;
    }

    public Manipulator() {
        manipulator = new TalonSRX(Wiring.MANIPULATOR_MOTOR);
        linkage = new TalonSRX(Wiring.LINKAGE_MOTOR);
        limitSwitch = new DigitalInput(0);

        manipulatorState = ManipulatorState.INACTIVE;
        previousState = ManipulatorState.INACTIVE;

        manipulator.setInverted(true);

        manipulator.configOpenloopRamp(.2, 0);

        manipulator.setNeutralMode(NeutralMode.Brake);

        flipper = new DoubleSolenoid(Wiring.FLIPPER_PCM_PORTA, Wiring.FLIPPER_PCM_PORTB);

        currentBreaker = new CurrentBreaker(Wiring.INTAKE_PDP_PORT, Calibration.INTAKE_MAX_CURRENT, 1000);

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

        linkage.configContinuousCurrentLimit(20);

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
        SmartDashboard.putString("Man State", manipulatorState.toString());

        if (SmartDashboard.getBoolean("Link TUNE", false)) {
			linkage.configMotionCruiseVelocity((int) SmartDashboard.getNumber("Link Vel", 0), 0);
			linkage.configMotionAcceleration((int) SmartDashboard.getNumber("Link Accel", 0), 0);
			linkage.config_kF(0, SmartDashboard.getNumber("Link F", 1.0), 0);
			linkage.config_kP(0, SmartDashboard.getNumber("Link P", 1.0), 0);
			linkage.config_kI(0, SmartDashboard.getNumber("Link I", 0), 0);
			linkage.config_kD(0, SmartDashboard.getNumber("Link D", 0), 0);
        }

        if (!linkageEncoderSet && !linkageEncoderSetting) {
			linkage.set(ControlMode.PercentOutput, .5);
			linkageEncoderSettingStartTime = System.currentTimeMillis();
			linkageEncoderSetting = true;
			System.out.println("calibrating linkage");
		}

		if (linkageEncoderSetting && (System.currentTimeMillis() >= (linkageEncoderSettingStartTime + 1000))) {
			linkage.getSensorCollection().setQuadraturePosition(0, 20);
			linkage.set(ControlMode.PercentOutput, 0);
			linkageEncoderSetting = false;
			linkageEncoderSet = true;
			System.out.println("calibrating linkage done");
        }
        
        if (intakeStalled() && manipulatorState == ManipulatorState.GETTING_CARGO) {
            holdCargo();
        } else if (!limitSwitch.get() && manipulatorState == ManipulatorState.GETTING_HATCH) {
            holdHatch();
        } else if (intakeStalled() && manipulatorState == ManipulatorState.GETTING_HATCH_FLOOR) {
            holdHatchFloor();
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
        lowerFlipper();
        linkage.set(ControlMode.MotionMagic, -975);
    }

    public static void linkageUp(){
        bringFlipperUp();
        linkage.set(ControlMode.MotionMagic, 0);
    }

    public static void resetLinkage() {
        linkageEncoderSet = false;
    }

    public static void intakeCargo() {
        linkageDown();
        lowerFlipper();
        Lift.goToStart();

        manipulatorState = ManipulatorState.GETTING_CARGO;
        manipulator.set(ControlMode.PercentOutput, -1);

        resetIntakeStallDetector();
        ejectEndTime = aDistantFutureTime();
    }

    public static void prepareToGetHatchFromFeeder() {
        manipulator.set(ControlMode.PercentOutput, -1);
        linkageDown();
        lowerFlipper();
        Lift.goHatchLvl1();
    }

    public static void intakeHatch() {
        Lift.getHatchPanel();
        
        manipulatorState = ManipulatorState.GETTING_HATCH;
    }

    public static void intakeHatchFloor() {
        linkageDown();
        lowerFlipper();
        Lift.goToStart();

        manipulatorState = ManipulatorState.GETTING_HATCH_FLOOR;
        manipulator.set(ControlMode.PercentOutput, 1);

        resetIntakeStallDetector();
        ejectEndTime = aDistantFutureTime();
    }

    public static void lowerFlipper() {
        flipper.set(Value.kForward);
    }

    public static void bringFlipperUp() {
        flipper.set(Value.kReverse);
    }

    private static void holdCargo() {
        manipulatorState = ManipulatorState.HOLDING_CARGO;
        resetIntakeStallDetector();
        // linkageUp();
        manipulator.set(ControlMode.PercentOutput, -.25);
    }

    private static void holdHatch() {
        manipulatorState = ManipulatorState.HOLDING_HATCH;
        manipulator.set(ControlMode.PercentOutput, -.25);
        // linkageUp();
    }

    public static boolean isHoldingCargo() {
        return manipulatorState == ManipulatorState.HOLDING_CARGO;
    }
    public static boolean isHoldingFloorHatch() {
        return manipulatorState == ManipulatorState.HOLDING_HATCH_FLOOR;
    }
    public static boolean isHoldingHatch() {
        return manipulatorState == ManipulatorState.HOLDING_HATCH;
    }

    private static void holdHatchFloor() {
        manipulatorState = ManipulatorState.HOLDING_HATCH_FLOOR;
        manipulator.set(ControlMode.PercentOutput, .15);
        resetIntakeStallDetector();
        Lift.goHatchLvl1();
        bringFlipperUp();
    }

    public static void holdGamePieceOverride() {
        if (manipulatorState == ManipulatorState.GETTING_CARGO) {
            holdCargo();
        } else if (manipulatorState == ManipulatorState.GETTING_HATCH) {
            holdHatch();
        } else if (manipulatorState == ManipulatorState.GETTING_HATCH_FLOOR) {
            holdHatchFloor();
        }
    }

    public static void ejectGamePiece() {
        ManipulatorState state = manipulatorState != ManipulatorState.INACTIVE ? manipulatorState : previousState;
        previousState = state;

        if (state == ManipulatorState.HOLDING_CARGO) {
            manipulator.set(ControlMode.PercentOutput, 1);
        } else if (state == ManipulatorState.HOLDING_HATCH) {
            Lift.scoreHatchPanel();
        } else if (state == ManipulatorState.HOLDING_HATCH_FLOOR) {
            manipulator.set(ControlMode.PercentOutput, -.75);
            Lift.scoreHatchPanel();
        }

        manipulatorState = ManipulatorState.INACTIVE;
        resetIntakeStallDetector();
        ejectEndTime = System.currentTimeMillis() + 800;
    }

    public static void goToTravelPosition() {
        manipulatorState = ManipulatorState.INACTIVE;
        linkageUp();
        stopIntake();
    }

    public static void setLinkageForPlacement() {
        linkageDown();
        lowerFlipper();
    }

    public static void stopIntake() {
        manipulator.set(ControlMode.PercentOutput, 0);
        resetIntakeStallDetector();
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

    public static double getLinkageEncoderPosition() {
        return linkage.getSensorCollection().getQuadraturePosition();
    }

}
