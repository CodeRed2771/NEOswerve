package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.libs.CurrentBreaker;

public class Manipulator { 
    private static Manipulator instance;
    private static TalonSRX manipulator;
    private static CurrentBreaker currentBreaker;
    private static CurrentBreaker linkageCurrentBreaker;
    private static TalonSRX linkage;
    private static DoubleSolenoid finger; 

    private static boolean linkageEncoderSet = false;
	private static boolean linkageEncoderSetting = false;
	private static double linkageEncoderSettingStartTime = System.currentTimeMillis();

    private enum ManipulatorState {
        INACTIVE,
        GETTING_CARGO,
        GETTING_HATCH,
        HOLDING_CARGO,
        HOLDING_HATCH,
    }

    private static ManipulatorState manipulatorState;
    private static ManipulatorState previousState;

    private static boolean linkageIsDown = false;

    private static double ejectEndTime;

    public static Manipulator getInstance() {
        if (instance == null)
            instance = new Manipulator();
        return instance;
    }

    public Manipulator() {
        manipulator = new TalonSRX(Wiring.MANIPULATOR_MOTOR);
        linkage = new TalonSRX(Wiring.LINKAGE_MOTOR);
        linkage.configFactoryDefault();
        

        manipulatorState = ManipulatorState.HOLDING_HATCH;
        previousState = ManipulatorState.HOLDING_HATCH;

        manipulator.setInverted(true);

        manipulator.configOpenloopRamp(.2, 0);

        manipulator.setNeutralMode(NeutralMode.Brake);

        finger = new DoubleSolenoid(Wiring.FLIPPER_PCM_PORTA, Wiring.FLIPPER_PCM_PORTB);

        currentBreaker = new CurrentBreaker(Wiring.INTAKE_PDP_PORT, Calibration.INTAKE_MAX_CURRENT, 150);
        
        linkageCurrentBreaker = new CurrentBreaker(Wiring.LINKAGE_PDP_PORT, 15, 2000);

        resetIntakeStallDetector();

        ejectEndTime = aDistantFutureTime();

        /* first choose the sensor */
		linkage.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);

		/* set the relevant frame periods to be at least as fast as periodic rate */
		linkage.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 10);
		linkage.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 10);

		/* set the peak and nominal outputs */
		linkage.configNominalOutputForward(0, 10);
		linkage.configNominalOutputReverse(0, 10);
		linkage.configPeakOutputForward(1, 10);
        linkage.configPeakOutputReverse(-1, 10);

        linkage.configContinuousCurrentLimit(20,10);

		linkage.setNeutralMode(NeutralMode.Brake);

		linkage.configClosedloopRamp(.25, 10);

		/* set closed loop gains in slot0 - see documentation */
		linkage.selectProfileSlot(0, 0);

		linkage.config_kP(0, Calibration.LINKAGE_P, 10);
		linkage.config_kI(0, Calibration.LINKAGE_I, 10);
		linkage.config_kD(0, Calibration.LINKAGE_D, 10);
		linkage.config_kF(0, Calibration.LINKAGE_F, 10);

		SmartDashboard.putNumber("Link Vel", Calibration.LINKAGE_VELOCITY);
		SmartDashboard.putNumber("Link Accel", Calibration.LINKAGE_ACCEL);
		SmartDashboard.putNumber("Link P", Calibration.LINKAGE_P);
		SmartDashboard.putNumber("Link I", Calibration.LINKAGE_I);
		SmartDashboard.putNumber("Link D", Calibration.LINKAGE_D);
		SmartDashboard.putNumber("Link F", Calibration.LINKAGE_F);

		SmartDashboard.putBoolean("Link TUNE", true);
    }

    public static void tick() {

        SmartDashboard.putString("Man State", manipulatorState.toString());

        // if (SmartDashboard.getBoolean("Link TUNE", false)) {
            SmartDashboard.putNumber("Link Enc", linkage.getSensorCollection().getQuadraturePosition());
            SmartDashboard.putNumber("Link Err", linkage.getClosedLoopError());
   
            linkage.configMotionCruiseVelocity((int) SmartDashboard.getNumber("Link Vel", 0), 10);
			linkage.configMotionAcceleration((int) SmartDashboard.getNumber("Link Accel", 0), 10);
			linkage.config_kF(0, SmartDashboard.getNumber("Link F", 1.0), 10);
			linkage.config_kP(0, SmartDashboard.getNumber("Link P", 1.0), 10);
			linkage.config_kI(0, SmartDashboard.getNumber("Link I", 0), 10);
			linkage.config_kD(0, SmartDashboard.getNumber("Link D", 0), 10);
        // }

        if (!linkageEncoderSet && !linkageEncoderSetting) {
			linkage.set(ControlMode.PercentOutput, .5);
			linkageEncoderSettingStartTime = System.currentTimeMillis();
			linkageEncoderSetting = true;
			System.out.println("calibrating linkage");
		}

		if (linkageEncoderSetting && (System.currentTimeMillis() >= (linkageEncoderSettingStartTime + 1000))) {
			linkage.getSensorCollection().setQuadraturePosition(0, 10);
			linkage.set(ControlMode.PercentOutput, 0);
			linkageEncoderSetting = false;
            linkageEncoderSet = true;
            linkageIsDown = false;
			System.out.println("calibrating linkage done");
        }
        
        if (intakeStalled() && manipulatorState == ManipulatorState.GETTING_CARGO) {
            holdCargo();
        } 

        if (System.currentTimeMillis() > ejectEndTime) {
            stopIntake();
            ejectEndTime = aDistantFutureTime();
        }

        if (linkage.getControlMode() == ControlMode.MotionMagic)
			SmartDashboard.putNumber("Link Setpt", linkage.getClosedLoopTarget());
		else
            SmartDashboard.putNumber("Link Setpt", -1);

        if (linkageStalled()) {
            linkage.set(ControlMode.PercentOutput, 0);
            System.out.println("LIFT CIRCUIT BREAKER TRIPPED");
            linkageCurrentBreaker.reset();
        }
            
    }

    public static boolean linkageStalled() {
        return (linkageCurrentBreaker.tripped());
	}

    // CONTROL METHODS ------------------------------------------------
    public static void linkageMove(double speed) {
        linkage.set(ControlMode.PercentOutput, speed);
    }
    
    public static void linkageDown() {
        // if (linkageIsDown) {
        //     linkage.set(ControlMode.PercentOutput, 0);
        // } else {
            linkage.set(ControlMode.Position, -1250);
            linkageIsDown = true;
            // linkage.set(ControlMode.MotionMagic, -900);
        //}
    }

    public static void linkageUp(){
        moveFingerUp();
        if (linkageIsDown) {
            linkage.set(ControlMode.Position, 0);
            linkageIsDown = false;
            // linkage.set(ControlMode.MotionMagic, 0);
        }
        else
            linkage.set(ControlMode.PercentOutput, 0);
    }

    public static void resetLinkage() {
        linkageEncoderSet = false;
    }

    public static void intakeCargo() {
        linkageDown();
        moveFingerUp();
        Lift.goToStart();

        manipulatorState = ManipulatorState.GETTING_CARGO;
        manipulator.set(ControlMode.PercentOutput, -1);

        resetIntakeStallDetector();
        ejectEndTime = aDistantFutureTime();
    }

    public static void prepareToGetHatchFromFeeder() {
        linkageDown();
        manipulator.set(ControlMode.PercentOutput, -1);
        moveFingerDown();
        Lift.goHatchLvl1();
    }

    public static void intakeHatch() {
        Lift.getHatchPanel();
        moveFingerDown();
        manipulatorState = ManipulatorState.GETTING_HATCH;
    }

    public static void moveFingerUp() {
        finger.set(Value.kReverse);
    }

    public static void moveFingerDown() {
        finger.set(Value.kForward);
    }

    public static DoubleSolenoid.Value fingerUp() {
        return Value.kReverse;
    }

    public static DoubleSolenoid.Value fingerDown() {
        return Value.kForward;
    }

    public static void moveFinger() {
        if (finger.get() == fingerDown()) {
            moveFingerUp();
        } else if (finger.get()  == fingerUp()) {
            moveFingerDown();
        }
    }

    private static void holdCargo() {
        manipulatorState = ManipulatorState.HOLDING_CARGO;
        resetIntakeStallDetector();
        manipulator.set(ControlMode.PercentOutput, 0);
    }

    private static void holdHatch() {
        manipulatorState = ManipulatorState.HOLDING_HATCH;
        manipulator.set(ControlMode.PercentOutput, 0);
        moveFingerUp();
    }

    public static boolean isHoldingCargo() {
        return manipulatorState == ManipulatorState.HOLDING_CARGO;
    }
    public static boolean isHoldingHatch() {
        return manipulatorState == ManipulatorState.HOLDING_HATCH;
    }

    public static void holdGamePieceOverride() {
        if (manipulatorState == ManipulatorState.GETTING_CARGO) {
            holdCargo();
        } else if (manipulatorState == ManipulatorState.GETTING_HATCH) {
            holdHatch();
            moveFingerUp();
        } 
    }

    public static void ejectGamePiece() {
        ManipulatorState state = manipulatorState != ManipulatorState.INACTIVE ? manipulatorState : previousState;
        previousState = state;

        if (state == ManipulatorState.HOLDING_CARGO) {
            manipulator.set(ControlMode.PercentOutput, 1);
        } else if (state == ManipulatorState.HOLDING_HATCH) {
            moveFingerDown();
        } 

        manipulatorState = ManipulatorState.INACTIVE;
        resetIntakeStallDetector();
        ejectEndTime = System.currentTimeMillis() + 800;
    }

    public static void setLinkageForPlacement() {
        linkageDown();
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
