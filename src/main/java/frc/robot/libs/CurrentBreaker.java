package frc.robot.libs;

import frc.robot.libs.SettableController;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CurrentBreaker {

	SettableController sc;
	PowerDistributionPanel pdp;
	int portnum;
	double currentThreshold;
	int timeOut;
	boolean tripped = false;
	long trippedTime = -1;
	long ignoreTime = -1;
	long ignoreDuration;

	public CurrentBreaker(SettableController sc, int portnum, double currentThreshold, int timeOut,	int ignoreDuration) {
		pdp = new PowerDistributionPanel();
		this.sc = sc == null ? new NullController() : sc;
		this.portnum = portnum;
		this.currentThreshold = currentThreshold;
		this.timeOut = timeOut;
		this.ignoreDuration = ignoreDuration;
	}

	public CurrentBreaker(SettableController sc, int portnum, double currentThreshold, int timeOut) {
		this(sc, portnum, currentThreshold, timeOut, -1);
	}

	public void checkCurrent() {
		SmartDashboard.putNumber("Current", pdp.getCurrent(portnum));
		SmartDashboard.putBoolean("Tripped", tripped);
		SmartDashboard.putNumber("Tripped Time", trippedTime);
		SmartDashboard.putNumber("Current Time", System.currentTimeMillis());
		
		if (System.currentTimeMillis() > ignoreTime) {
			if (pdp.getCurrent(portnum) > currentThreshold && trippedTime == -1) {
				trippedTime = System.currentTimeMillis() + timeOut;
			}
			if (trippedTime != -1 && System.currentTimeMillis() >= trippedTime) {
				tripped = true;
				sc.set(0.0);
			}
		}
	}

	public boolean tripped() {
		checkCurrent();
		return tripped;
	}

	public void set(double speed) {
		checkCurrent();
		if (!tripped) {
			sc.set(speed);
		}
	}

	public void reset() {
		tripped = false;
		trippedTime = -1;
		if (ignoreDuration != -1) {
			ignoreTime = System.currentTimeMillis() + ignoreDuration;
//			 SmartDashboard.putNumber("ignoreDuration", ignoreDuration);
//			 SmartDashboard.putNumber("IgnoreTime", ignoreTime);
			// Logger.getInstance().log(Logger.Level.INFO,1,"ignoreDuration is
			// "+ignoreDuration);
			// Logger.getInstance().log(Logger.Level.INFO, 1, "Ignore Time Reset
			// to "+ignoreTime);
		}
	}

	public void ignoreFor(long time) {
		ignoreDuration = time;
	}

	public double getCurrent() {
		return pdp.getCurrent(portnum);
	}

	private class NullController implements SettableController {

		@Override
		public void set(double value) {

		}

	}
}
