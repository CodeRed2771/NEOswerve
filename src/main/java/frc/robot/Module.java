package frc.robot;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Module {

	public WPI_TalonSRX driveMotor, turnMotor;
	private final int FULL_ROTATION_TICKS = 4096;
	private double TURN_P, TURN_I, TURN_D, DRIVE_P, DRIVE_I, DRIVE_D;
	private int TURN_IZONE, DRIVE_IZONE;
	private double mTurnZeroPos = 0;
	private double mCurrentDriveSetpoint = 0;
	private boolean mIsReversed = false;

	/**
	 * Make a swerve module
	 * 
	 * @param driveTalonID Talon we are using for driving
	 * @param turnTalonID  Talon we are using to turn
	 * @param tP           P constant for the turning PID
	 * @param tI           I constant for the turning PID
	 * @param tD           D constant for the turning PID
	 * @param tIZone       I Zone value for the turning PID
	 */
	public Module(int driveTalonID, int turnTalonID, double dP, double dI, double dD, int dIZone, double tP, double tI,
			double tD, int tIZone, double tZeroPos) {

		driveMotor = new WPI_TalonSRX(driveTalonID);

		driveMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
		DRIVE_P = dP;
		DRIVE_I = dI;
		DRIVE_D = dD;
		DRIVE_IZONE = dIZone;

		driveMotor.config_kP(0, DRIVE_P, 10);
		driveMotor.config_kI(0, DRIVE_I, 10);
		driveMotor.config_kD(0, DRIVE_D, 10);
		driveMotor.config_IntegralZone(0, DRIVE_IZONE, 10);
		driveMotor.selectProfileSlot(0, 0);

		driveMotor.configOpenloopRamp(.1, 10);
		driveMotor.configClosedloopRamp(.05, 10);

		driveMotor.configMotionCruiseVelocity(Calibration.DT_MM_VELOCITY, 10);
		driveMotor.configMotionAcceleration(Calibration.DT_MM_ACCEL, 10);
		driveMotor.setSensorPhase(true);

		turnMotor = new WPI_TalonSRX(turnTalonID);
		mTurnZeroPos = tZeroPos;

		turnMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
		TURN_P = tP;
		TURN_I = tI;
		TURN_D = tD;
		TURN_IZONE = tIZone;

		turnMotor.config_kP(0, TURN_P, 10);
		turnMotor.config_kI(0, TURN_I, 10);
		turnMotor.config_kD(0, TURN_D, 10);
		turnMotor.config_IntegralZone(0, TURN_IZONE, 10);
		turnMotor.selectProfileSlot(0, 0);

		turnMotor.configClosedloopRamp(.1, 10);
	}

	public void setFollower(int talonToFollow) {
		if (talonToFollow != 0) {
			driveMotor.set(ControlMode.Follower, talonToFollow);
		} else
			driveMotor.set(ControlMode.Velocity, 0);
	}

	public void setDriveMMAccel(int accel) {
		driveMotor.configMotionAcceleration(accel, 0);
	}

	public void setDriveMMVelocity(int velocity) {
		driveMotor.configMotionCruiseVelocity(velocity, 0);
	}

	public int getDriveVelocity() {
		return driveMotor.getSelectedSensorVelocity(0);
	}

	/**
	 * Setting turn motor power
	 * 
	 * @param power value from -1 to 1
	 */
	public void setTurnPower(double power) {
		this.turnMotor.set(ControlMode.PercentOutput, power);
	}

	/**
	 * Setting drive motor power
	 * 
	 * @param power value from -1 to 1
	 */
	public void setDrivePower(double power) {
		this.driveMotor.set((getIsModuleReversed() ? -1 : 1) * power);
	}

	/**
	 * Gets the absolute encoder position for the turn encoder It will be a value
	 * between 0 and 1
	 * 
	 * @return turn encoder absolute position
	 */
	public double getTurnAbsolutePosition() {
		return (turnMotor.getSensorCollection().getPulseWidthPosition() & 0xFFF) / 4095d;
	}

	public double getTurnRelativePosition() {
		// returns the 0 to 1 value of the turn position
		// relative to the calibrated zero position.
		// if our current reading is .600 and calibration is .200
		// then our relative position is .400

		double currentPos = getTurnAbsolutePosition();
		if (currentPos - getTurnZeroPosition() > 0) { // turnZeroPos is calibrated zero position
			return currentPos - getTurnZeroPosition();
		} else {
			return (1 - getTurnZeroPosition()) + currentPos;
		}
	}

	public double getTurnAngle() {
		// returns the angle in -180 to 180 range
		double turnPos = getTurnRelativePosition();
		if (turnPos > .5) {
			return (360 - (turnPos * 360));
		} else
			return turnPos * 360;
	}

	/**
	 * @return calibrated zero encoder position - 0 to 1
	 */
	public double getTurnZeroPosition() {
		return mTurnZeroPos;
	}

	public boolean getIsModuleReversed() {
		return mIsReversed;
	}

	public int getDriveEncoder() {
		return driveMotor.getSelectedSensorPosition(0);
	}

	public void resetDriveEncoder() {
		driveMotor.getSensorCollection().setQuadraturePosition(0, 0);
	}

	/**
	 * Getting the turn encoder position (not absolute)
	 * 
	 * @return turn encoder position
	 */
	public int getTurnEncoderValue() {
		return turnMotor.getSelectedSensorPosition(0);
	}

	public void resetTurnEncoder() {
		turnMotor.getSensorCollection().setQuadraturePosition(0, 0);
	}

	public void setTurnEncoderValue(int d) {
		turnMotor.getSensorCollection().setQuadraturePosition(d, 0);
	}

	/**
	 * TO DO - This has not been updated
	 * 
	 * @return true if the encoder is connected
	 */
	public boolean isTurnEncConnected() {
		// return turn.isSensorPresent(FeedbackDevice.CtreMagEncoder_Relative) ==
		// FeedbackDeviceStatus.FeedbackStatusPresent;
		return true; // didn't immediately see a compatible replacement
	}

	public int getTurnRotations() {
		// note that casting to int truncates the decimal portion
		// it does not round. So 3.999 will be 3, -3.99 will be -3
		return (int) (turnMotor.getSelectedSensorPosition(0) / FULL_ROTATION_TICKS);
	}

	public double getTurnOrientation() {
		return (turnMotor.getSelectedSensorPosition(0) % FULL_ROTATION_TICKS) / FULL_ROTATION_TICKS;
	}

	// These are used for driving and turning in auto.
	public void setDrivePIDToSetPoint(double setpoint) {
		mCurrentDriveSetpoint = setpoint;
		driveMotor.set(ControlMode.MotionMagic, setpoint);
	}

	public boolean hasDriveCompleted(int allowedError) {
		return Math.abs(mCurrentDriveSetpoint - getDriveEncoder()) <= allowedError;
	}

	public boolean hasDriveCompleted() {
		return hasDriveCompleted(0);
	}

	public void setTurnPIDToSetPoint(double setpoint) {
		turnMotor.set(ControlMode.Position, setpoint);
	}

	public void setTurnOrientation(double requestedTurnPosition) {
		setTurnOrientation(requestedTurnPosition, true);
	}

	/**
	 * Set turn to pos from 0 to 1
	 * 
	 * @param requestedTurnPosition value between 0 and 1 to indicate how module
	 *                              should be turned
	 * @param optimizeTurn          indicates whether to use shortest rotation of
	 *                              module and invert drive direction
	 */
	public void setTurnOrientation(double requestedTurnPosition, boolean optimizeTurn) {
		double distanceToRequestedPosition = 0;
		double distanceToOppositePosition = 0;
		double closestTurnPosition = 0;
		int newEncoderSetpoint = 0;
		int currentTurnEncoderTicks = getTurnEncoderValue();
		double currentTurnPosition = getTurnRelativePosition(); // 0 to 1 of our current position
		double oppositeTurnPosition = (requestedTurnPosition + 0.5) % 1.0;

		if (optimizeTurn) {
			if (Math.abs(currentTurnPosition - requestedTurnPosition) > .5)
				distanceToRequestedPosition = 1 - (Math.abs(currentTurnPosition - requestedTurnPosition));
			else
				distanceToRequestedPosition = Math.abs(currentTurnPosition - requestedTurnPosition);

			if (Math.abs(currentTurnPosition - oppositeTurnPosition) > .5)
				distanceToOppositePosition = 1 - (Math.abs(currentTurnPosition - oppositeTurnPosition));
			else
				distanceToOppositePosition = Math.abs(currentTurnPosition - oppositeTurnPosition);

			// see which turn position is closest to where we are.
			closestTurnPosition = distanceToOppositePosition < distanceToRequestedPosition ? oppositeTurnPosition
					: requestedTurnPosition;
		} else {
			closestTurnPosition = requestedTurnPosition;
		}

		// if the closestTurnPosition is not the requested turn position, then we need
		// the drive code to know to reverse itself when driving, so set the class level
		// flag isReversed.
		mIsReversed = closestTurnPosition != requestedTurnPosition;

		// Now we need to take that "closest turn position", which indicates how far
		// into one revolution we need to be, and figure out the relative encoder
		// value to get there.
		// The relative encoder value will have a value that could indicate many
		// revolutions and therefore we need to figure out which revolution we're on,
		// then add in the appropriate ticks to get to our desired position within that
		// revolution.  In some cases it will be shorter to unwind to the previous
		// revolution.  e.g. We're at rotation 3.1 and need to be at a .9 position, 
		// then it's better to go to 2.9 then 3.9.

		// turnTicks will be the amount of ticks relative to our current position
		// turnTicks is not optimized for direction yet. That's in the next part. 
		int turnTicks = (int)((closestTurnPosition - currentTurnPosition) * FULL_ROTATION_TICKS);
	
		// now see if we're better off rotating forward or backward to get to our
		// desired position.
		if (Math.abs(turnTicks) <= (FULL_ROTATION_TICKS / 2)) {
			// we're within a half rotation, so we can simply add the ticks 
			// (which may also be negative) to the current encoder position
			newEncoderSetpoint = currentTurnEncoderTicks + turnTicks;
		} else {
			// we're turning more than half a rotation
			// so we'll go back the other way instead
			newEncoderSetpoint = currentTurnEncoderTicks - (FULL_ROTATION_TICKS - turnTicks);
		}

		// now send the new setpoint to the motor
		turnMotor.set(ControlMode.Position, newEncoderSetpoint);
	}

	public double getTurnError() {
		return turnMotor.getClosedLoopError(0);
	}

	public double getDriveError() {
		// note that when using Motion Magic, the error is not what you'd expect
		// MM sets intermediate set points, so the error is just the error to
		// that set point, not to the final setpoint.
		return driveMotor.getClosedLoopError(0);
	}

	public void stopDriveAndTurnMotors() {
		setDrivePower(0);
		setTurnPower(0);
	}

	public void stopDrive() {
		setDrivePower(0);
	}

	public void setBrakeMode(boolean b) {
		driveMotor.setNeutralMode(b ? NeutralMode.Brake : NeutralMode.Coast);
	}

	public void setDrivePIDValues(double p, double i, double d) {
		driveMotor.config_kP(0, p, 0);
		driveMotor.config_kI(0, i, 0);
		driveMotor.config_kD(0, d, 0);
	}

	public void setTurnPIDValues(double p, double i, double d) {
		turnMotor.config_kP(0, p, 0);
		turnMotor.config_kI(0, i, 0);
		turnMotor.config_kD(0, d, 0);
	}

}