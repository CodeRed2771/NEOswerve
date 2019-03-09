package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Lift {
	private static Lift instance;
	private static TalonSRX liftMotor;

	public static Lift getInstance() {
		if (instance == null)
			instance = new Lift();
		return instance;
	}

	public Lift() {
		liftMotor = new TalonSRX(Wiring.LIFT_MASTER);

		/* first choose the sensor */
		liftMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);

		/* set the relevant frame periods to be at least as fast as periodic rate */
		liftMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 0);
		liftMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 0);

		/* set the peak and nominal outputs */
		liftMotor.configNominalOutputForward(0, 0);
		liftMotor.configNominalOutputReverse(0, 0);
		liftMotor.configPeakOutputForward(1, 0);
		liftMotor.configPeakOutputReverse(-1, 0);
		liftMotor.setNeutralMode(NeutralMode.Brake);

		liftMotor.configClosedloopRamp(.25, 0);

		/* set closed loop gains in slot0 - see documentation */
		liftMotor.selectProfileSlot(0, 0);

		liftMotor.config_kP(0, Calibration.LIFT_P, 0);
		liftMotor.config_kI(0, Calibration.LIFT_I, 0);
		liftMotor.config_kD(0, Calibration.LIFT_D, 0);
		liftMotor.config_kF(0, Calibration.LIFT_F, 0);

		/* set acceleration and vcruise velocity - see documentation */
		liftMotor.configMotionCruiseVelocity(Calibration.LIFT_VELOCITY, 0);
		liftMotor.configMotionAcceleration(Calibration.LIFT_ACCEL, 0);

		/* zero the sensor */
		liftMotor.setSelectedSensorPosition(0, 0, 0);

		SmartDashboard.putNumber("Lift Vel", Calibration.LIFT_VELOCITY);
		SmartDashboard.putNumber("Lift Accel", Calibration.LIFT_ACCEL);
		SmartDashboard.putNumber("Lift P", Calibration.LIFT_P);
		SmartDashboard.putNumber("Lift I", Calibration.LIFT_I);
		SmartDashboard.putNumber("Lift D", Calibration.LIFT_D);
		SmartDashboard.putNumber("Lift F", Calibration.LIFT_F);

		SmartDashboard.putBoolean("Lift TUNE", false);
	}

	public static void tick() {

		SmartDashboard.putNumber("lift cur", liftMotor.getOutputCurrent());
		
		if (SmartDashboard.getBoolean("Lift TUNE", false)) {
			liftMotor.configMotionCruiseVelocity((int) SmartDashboard.getNumber("Lift Vel", 0), 0);
			liftMotor.configMotionAcceleration((int) SmartDashboard.getNumber("Lift Accel", 0), 0);
			liftMotor.config_kF(0, SmartDashboard.getNumber("Lift F", 1.0), 0);
			liftMotor.config_kP(0, SmartDashboard.getNumber("Lift P", 1.0), 0);
			liftMotor.config_kI(0, SmartDashboard.getNumber("Lift I", 0), 0);
			liftMotor.config_kD(0, SmartDashboard.getNumber("Lift D", 0), 0);
		}

		SmartDashboard.putNumber("Lift Enc", liftMotor.getSensorCollection().getQuadraturePosition());
		if (liftMotor.getControlMode() == ControlMode.MotionMagic)
			SmartDashboard.putNumber("Lift Setpt", liftMotor.getClosedLoopTarget());
		else
			SmartDashboard.putNumber("Lift Setpt", -1);
	}

	public static void move(double speed) {
		// limit speed
		if (speed < -.3)
			speed = -.3;
		if (speed > .3)
			speed = .3;

		liftMotor.set(ControlMode.PercentOutput, speed);
	}

	public static void moveSetpoint(double direction) {
		int newSetpoint;

		if (direction > 0) {
			newSetpoint = liftMotor.getSelectedSensorPosition(0) + 4000;
			if (newSetpoint >= 0) {
				newSetpoint = 0;
			}
		} else {
			newSetpoint = liftMotor.getSelectedSensorPosition(0) - 4000;
			if (newSetpoint < -40000) {
				newSetpoint = -40000;
			}
		}

		liftMotor.set(ControlMode.MotionMagic, newSetpoint);
	}

	public static void goToStart() {
		liftMotor.set(ControlMode.MotionMagic, 0);
	}

	private static final double HATCH_ACQUIRING = -2000; // WARNING THIS IS WRONG
	private static final double HATCH_SCORING = -200;
	private static final double HATCH_LEVEL_1 = -1340;
	private static final double HATCH_LEVEL_2 = -11000;
	private static final double HATCH_LEVEL_3 = -22000;
	private static final double CARGO_LEVEL_1 = -4340;
	private static final double CARGO_LEVEL_2 = -14000;
	private static final double CARGO_LEVEL_3 = -25000;

	public static void getHatchPanel() {
		liftMotor.set(ControlMode.MotionMagic, HATCH_ACQUIRING);
	}

	public static void scoreHatchPanel() {
		liftMotor.set(ControlMode.MotionMagic, liftMotor.getSensorCollection().getQuadraturePosition() - HATCH_SCORING);
	}
	
	public static void goHatchLvl1() {
		liftMotor.set(ControlMode.MotionMagic, HATCH_LEVEL_1);
	}

	public static void goCargoLvl1() {
		liftMotor.set(ControlMode.MotionMagic, CARGO_LEVEL_1);
	}

	public static void goHatchLvl2() {
		liftMotor.set(ControlMode.MotionMagic, HATCH_LEVEL_2);
	}

	public static void goCargoLvl2() {
		liftMotor.set(ControlMode.MotionMagic, CARGO_LEVEL_2);
	}

	public static void goHatchLvl3() {
		liftMotor.set(ControlMode.MotionMagic, HATCH_LEVEL_3);
	}

	public static void goCargoLvl3() {
		liftMotor.set(ControlMode.MotionMagic, CARGO_LEVEL_3);
	}

	// This here may not be right we assumued that this is just hatch level one so
	// we put the variable in it has 'HATCH_LEVEL_1'
	public static void goCargoShipCargo() {
		liftMotor.set(ControlMode.MotionMagic, HATCH_LEVEL_3);
	}

	public static void stop() {
		liftMotor.set(ControlMode.PercentOutput, 0);
	}

	// returns true if the lift is high enough that we should reduce drving speed
	public static boolean driveCautionNeeded() {
		return Math.abs(liftMotor.getSensorCollection().getPulseWidthPosition()) > 15000;
	}
}
