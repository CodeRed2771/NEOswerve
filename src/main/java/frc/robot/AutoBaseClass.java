package frc.robot;

import frc.robot.libs.Timer;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public abstract class AutoBaseClass {
	private Timer mAutoTimer;		// note that the timer is ticked in isRunning() and hasCompleted()
	private char mRobotPosition;
	private boolean mIsRunning = false;
	
	public AutoBaseClass(char robotPosition) {
		mRobotPosition = robotPosition;
		mAutoTimer = new Timer();
	}

	public AutoBaseClass() {
		this('X');
	}

	public abstract void tick();

	public void start() {
		mAutoTimer.setStep(0);
		mIsRunning = true;

		DriveAuto.reset();
		
        // set current position equal to Gyro
        // otherwise turns will be relative to 0
        DriveAuto.setTurnDegreesToCurrentAngle(); 
	}

	public void stop() {
		mIsRunning = false;
		DriveAuto.stop();
	}

	public boolean isRunning() {
		mAutoTimer.tick();  // we need to tick the timer and this is a good place to do it.
		DriveAuto.tick();
		return mIsRunning;
	}
	
	public boolean hasCompleted() {
		mAutoTimer.tick();  // we need to tick the timer and this is a good place to do it.
		return !mIsRunning;
	}

	public int getCurrentStep() {
		return mAutoTimer.getStep();
	}

	public void setStep(int step) {
		mAutoTimer.setStep(step);
	}

	public double getStepTimeRemainingInSeconds() {
		return mAutoTimer.getTimeRemainingSeconds();
	}

	public double getStepTimeRemainingInMilliSeconds() {
		return mAutoTimer.getTimeRemainingMilliseconds();
	}

	public void driveInches(double distance, double angle, double maxPower) {
		DriveAuto.driveInches(distance, angle, maxPower);
	}
	
	public boolean driveCompleted() {
		return DriveAuto.hasArrived();
	}
	
	public boolean turnCompleted() {
		return DriveAuto.turnCompleted();
	}
	public boolean turnCompleted(double allowedErrorDegrees) {
		return DriveAuto.turnCompleted(allowedErrorDegrees);
	}

	public void turnDegrees(double degrees, double maxPower) {
		DriveAuto.turnDegrees(degrees, maxPower);
	}

	public void turnDegreesV2(double degrees, double maxPower) {
		DriveAuto.turnDegreesV2(degrees, maxPower);
	}

//	public void continuousTurn(double degrees, double maxPower) {
//		DriveAuto.continuousTurn(degrees, maxPower);
//	}
	 
	public void continuousDrive(double inches, double maxPower) {
		DriveAuto.continuousDrive(inches, maxPower);
	}
	
	public char robotPosition() {
		return mRobotPosition;
	}

	public void advanceStep() {
		mAutoTimer.stopTimerAndAdvanceStep();
	}

	// starts a timer for the time indicated and then immediately advances the
	// stage counter
	// this is typically used when starting a driving maneuver because the next
	// stage would
	// be watching to see when the maneuver was completed.
	public void setTimerAndAdvanceStep(long milliseconds) {
		mAutoTimer.setTimerAndAdvanceStep(milliseconds);
	}
	
	public void setTimer(long milliseconds) {
		mAutoTimer.setTimer(milliseconds);
	}
	
	public boolean timeExpired() {
		return mAutoTimer.timeExpired();
	}
	
}
