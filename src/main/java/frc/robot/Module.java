// THIS IS THE MODULE THAT WAS USED IN CANADA AND THAT WE HAD AT THE END OF CANADA.
// CS 3/21/2019

package frc.robot;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Module {
	public WPI_TalonSRX drive, turn;
	private char mModuleID;
	private final int FULL_ROTATION = 4096;
	private double TURN_P, TURN_I, TURN_D, DRIVE_P, DRIVE_I, DRIVE_D;
	private final int TURN_IZONE, DRIVE_IZONE;
	private double turnZeroPos = 0;
	private double currentDriveSetpoint = 0;
	private boolean isReversed = false;

	/**
	 * Lets make a new module :)
	 * 
	 * @param driveTalonID First I gotta know what talon we are using for driving
	 * @param turnTalonID  Next I gotta know what talon we are using to turn
	 * @param tP           I probably need to know the P constant for the turning
	 *                     PID
	 * @param tI           I probably need to know the I constant for the turning
	 *                     PID
	 * @param tD           I probably need to know the D constant for the turning
	 *                     PID
	 * @param tIZone       I might not need to know the I Zone value for the turning
	 *                     PID
	 */
	public Module(int driveTalonID, int turnTalonID, double dP, double dI, double dD, int dIZone, double tP, double tI,
			double tD, int tIZone, double tZeroPos, char moduleID) {
		drive = new WPI_TalonSRX(driveTalonID);
		drive.configFactoryDefault(10);
		mModuleID = moduleID;
		drive.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0); // ?? don't know if zeros are right
		DRIVE_P = dP;
		DRIVE_I = dI;
		DRIVE_D = dD;
		DRIVE_IZONE = dIZone;

		drive.config_kP(0, DRIVE_P, 0);
		drive.config_kI(0, DRIVE_I, 0);
		drive.config_kD(0, DRIVE_D, 0);
		drive.config_IntegralZone(0, DRIVE_IZONE, 0);
		drive.selectProfileSlot(0, 0);

		drive.configOpenloopRamp(.1, 0);
		drive.configClosedloopRamp(.05, 0);

		drive.configMotionCruiseVelocity(Calibration.DT_MM_VELOCITY, 0);
		drive.configMotionAcceleration(Calibration.DT_MM_ACCEL, 0);
		drive.setSensorPhase(true);

		turn = new WPI_TalonSRX(turnTalonID);
		turn.configFactoryDefault(10);

		turnZeroPos = tZeroPos;

		turn.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0); // ?? don't know if zeros are right
		TURN_P = tP;
		TURN_I = tI;
		TURN_D = tD;
		TURN_IZONE = tIZone;

		turn.config_kP(0, TURN_P, 0);
		turn.config_kI(0, TURN_I, 0);
		turn.config_kD(0, TURN_D, 0);
		turn.config_IntegralZone(0, TURN_IZONE, 0);
		turn.selectProfileSlot(0, 0);

		turn.configClosedloopRamp(.1, 0);
	}

	public void setFollower(int talonToFollow) {
		if (talonToFollow != 0) {
			drive.set(ControlMode.Follower, talonToFollow);
		} else
			drive.set(ControlMode.Velocity, 0);
	}

	public void setDriveMMAccel(int accel) {
		drive.configMotionAcceleration(accel, 0);
	}

	public void setDriveMMVelocity(int velocity) {
		drive.configMotionCruiseVelocity(velocity, 0);
	}

	public int getDriveVelocity() {
		return drive.getSelectedSensorVelocity(0);
	}

	/**
	 * Setting turn motor power
	 * 
	 * @param p value from -1 to 1
	 */
	public void setTurnPower(double p) {
		this.turn.set(ControlMode.PercentOutput, p);
	}

	/**
	 * Setting drive motor power
	 * 
	 * @param p value from -1 to 1
	 */
	public void setDrivePower(double p) {
		this.drive.set((isReversed ? -1 : 1) * p);
	}

	/**
	 * Getting the turn encoder position (not absolute)
	 * 
	 * @return turn encoder position
	 */
	public int getTurnRelativePosition() {
		return turn.getSelectedSensorPosition(0);
	}

	/**
	 * Gets the absolute encoder position for the turn encoder It will be a value
	 * between 0 and 1
	 * 
	 * @return turn encoder absolute position
	 */
	public double getTurnAbsolutePosition() {
		return (turn.getSensorCollection().getPulseWidthPosition() & 0xFFF) / 4096d;
	}

	public double getTurnPosition() {
		// returns the 0 to 1 value of the turn position
		// uses the calibration value and the actual position
		// to determine the relative turn position

		double currentPos = getTurnAbsolutePosition();
		if (currentPos - turnZeroPos > 0) {
			return currentPos - turnZeroPos;
		} else {
			return (1 - turnZeroPos) + currentPos;
		}
	}

	public double getTurnAngle() {
		// returns the angle in -180 to 180 range
		double turnPos = getTurnPosition();
		if (turnPos > .5) {
			return (360 - (turnPos * 360));
		} else
			return turnPos * 360;
	}

	public boolean modulesReversed() {
		return isReversed;
	}

	public void unReverseModule() {
		isReversed = false;
	}

	public void resetTurnEnc() {
		this.turn.getSensorCollection().setQuadraturePosition(0, 0);
	}

	public int getDriveEnc() {
		return drive.getSelectedSensorPosition(0);
	}

	public void resetDriveEnc() {
		this.drive.getSensorCollection().setQuadraturePosition(0, 0);
	}

	public void setEncPos(int d) {
		turn.getSensorCollection().setQuadraturePosition(d, 0);
	}

	/**
	 * Is electrical good? Probably not.... Is the turn encoder connected?
	 * 
	 * @return true if the encoder is connected
	 */
	public boolean isTurnEncConnected() {
		// return turn.isSensorPresent(FeedbackDevice.CtreMagEncoder_Relative) ==
		// FeedbackDeviceStatus.FeedbackStatusPresent;
		return true; // didn't immediately see a compatible replacement
	}

	public int getTurnRotations() {
		return (int) (turn.getSelectedSensorPosition(0) / FULL_ROTATION);
	}

	public double getTurnOrientation() {
		return (turn.getSelectedSensorPosition(0) % FULL_ROTATION) / FULL_ROTATION;

		// SmartDashboard.putNumber("module-a-" + this.hashCode(),
		// turn.getSelectedSensorPosition(0));
		// SmartDashboard.putNumber("module-b-" + this.hashCode(),
		// turn.getSelectedSensorPosition(0) % FULL_ROTATION);
		// SmartDashboard.putNumber("module-c-" + this.hashCode(),
		// (turn.getSelectedSensorPosition(0) % FULL_ROTATION) / FULL_ROTATION);

	}

	// These are used for driving and turning in auto.
	public void setDrivePIDToSetPoint(double setpoint) {
		currentDriveSetpoint = setpoint;
		drive.set(ControlMode.MotionMagic, setpoint);

	}

	public boolean hasDriveCompleted(int allowedError) {
		return Math.abs(currentDriveSetpoint - getDriveEnc()) <= allowedError;
	}

	public boolean hasDriveCompleted() {
		return hasDriveCompleted(0);
	}

	public void setTurnPIDToSetPoint(double setpoint) {
		turn.set(ControlMode.Position, setpoint);
	}

	public void setTurnOrientation(double position) {
		setTurnOrientation(position, true);
	}

	/**
	 * Set turn to pos from 0 to 1 using PID
	 * 
	 * @param reqPosition orientation to set to
	 */
	public void setTurnOrientation(double reqPosition, boolean optimize) {
		int base = getTurnRotations() * FULL_ROTATION;
		double currentTurnPosition = getTurnPosition();
		double reverseTurnPosition = (reqPosition + 0.5) % 1.0;
		double distanceToNormalPosition;
		double distanceToReversePosition;
		double closestTurnPosition = 0;
		int turnRelativePosition = getTurnRelativePosition();
		// double distanceToNormalPosition = Math.abs(currentTurnPosition - position);
		// double distanceToReversePosition = Math.abs(currentTurnPosition -
		// reverseTurnPosition);

		if (currentTurnPosition - reqPosition >= 0)
			if (currentTurnPosition - reqPosition > .5)
				distanceToNormalPosition = (1 - currentTurnPosition) + reqPosition;
			else
				distanceToNormalPosition = currentTurnPosition - reqPosition;
		else if (reqPosition - currentTurnPosition > .5)
			distanceToNormalPosition = (1 - reqPosition) + currentTurnPosition;
		else
			distanceToNormalPosition = reqPosition - currentTurnPosition;

		if (currentTurnPosition - reverseTurnPosition >= 0)
			if (currentTurnPosition - reverseTurnPosition > .5)
				distanceToReversePosition = (1 - currentTurnPosition) + reverseTurnPosition;
			else
				distanceToReversePosition = currentTurnPosition - reverseTurnPosition;
		else if (reverseTurnPosition - currentTurnPosition > .5)
			distanceToReversePosition = (1 - reverseTurnPosition) + currentTurnPosition;
		else
			distanceToReversePosition = reverseTurnPosition - currentTurnPosition;

		if (optimize) {
			closestTurnPosition = distanceToReversePosition < distanceToNormalPosition ? reverseTurnPosition
					: reqPosition;
		} else
			closestTurnPosition = reqPosition;

		isReversed = closestTurnPosition != reqPosition;

		if (turnRelativePosition >= 0) {
			if ((base + (closestTurnPosition * FULL_ROTATION)) - turnRelativePosition < -FULL_ROTATION / 2) {
				base += FULL_ROTATION;
			} else if ((base + (closestTurnPosition * FULL_ROTATION)) - turnRelativePosition > FULL_ROTATION / 2) {
				base -= FULL_ROTATION;
			}
			// showDetailsOnDash(base, turnRelativePosition, currentTurnPosition, reqPosition, reverseTurnPosition,
			// 		distanceToNormalPosition, distanceToReversePosition, closestTurnPosition, optimize,
			// 		(int) (((closestTurnPosition * FULL_ROTATION) + (base))));
			turn.set(ControlMode.Position, (((closestTurnPosition * FULL_ROTATION) + (base))));
		} else {
			if ((base - ((1 - closestTurnPosition) * FULL_ROTATION)) - turnRelativePosition < -FULL_ROTATION / 2) {
				base += FULL_ROTATION;
			} else if ((base - ((1 - closestTurnPosition) * FULL_ROTATION)) - turnRelativePosition > FULL_ROTATION
					/ 2) {
				base -= FULL_ROTATION;
			}
			// showDetailsOnDash(base, turnRelativePosition, currentTurnPosition, reqPosition, reverseTurnPosition,
			// 		distanceToNormalPosition, distanceToReversePosition, closestTurnPosition, optimize,
			// 		(int) (base - (((1 - closestTurnPosition) * FULL_ROTATION))));
			turn.set(ControlMode.Position, (base - (((1 - closestTurnPosition) * FULL_ROTATION))));
		}
	}

	// private void showDetailsOnDash(int base, int turnRelative, double currentTurnPosition, double requestedPosition,
	// 		double reverseTurnPos, double distNormal, double distReverse, double closestTurn, boolean optimize,
	// 		int newSetpoint) {
	// 	if (mModuleID == 'B') {
	// 		System.out.println("CurrentTurn: " + currentTurnPosition + " Req Pos: " + requestedPosition + " dist Norm: "
	// 				+ distNormal + " dist Rev: " + distReverse);
	// 		SmartDashboard.putNumber("AAA Base", base);
	// 		SmartDashboard.putNumber("AAA turnRel", turnRelative);
	// 		SmartDashboard.putNumber("AAA req pos", requestedPosition);
	// 		SmartDashboard.putNumber("AAA cur pos", currentTurnPosition);
	// 		SmartDashboard.putNumber("AAA revPos", reverseTurnPos);
	// 		SmartDashboard.putNumber("AAA distnorm", distNormal);
	// 		SmartDashboard.putNumber("AAA distrev", distReverse);
	// 		SmartDashboard.putBoolean("AAA reversed", isReversed);
	// 		SmartDashboard.putNumber("AAA closest", closestTurn);
	// 		SmartDashboard.putNumber("AAA new set", newSetpoint);
	// 		SmartDashboard.putBoolean("AAA optimize", optimize);
	// 	}
	// }

	public double getTurnError() {
		return turn.getClosedLoopError(0);
	}

	public double getDriveError() {
		// note that when using Motion Magic, the error is not what you'd expect
		// MM sets intermediate set points, so the error is just the error to
		// that set point, not to the final setpoint.
		return drive.getClosedLoopError(0);
	}

	public void stopDriveAndTurnMotors() {
		setDrivePower(0);
		setTurnPower(0);
	}

	public void stopDrive() {
		setDrivePower(0);
	}

	public void setBrakeMode(boolean b) {
		drive.setNeutralMode(b ? NeutralMode.Brake : NeutralMode.Coast);
	}

	public void setDrivePIDValues(double p, double i, double d) {
		drive.config_kP(0, p, 0);
		drive.config_kI(0, i, 0);
		drive.config_kD(0, d, 0);
	}

	public void setTurnPIDValues(double p, double i, double d) {
		turn.config_kP(0, p, 0);
		turn.config_kI(0, i, 0);
		turn.config_kD(0, d, 0);
	}

}