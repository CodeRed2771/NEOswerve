package frc.robot.libs;

import edu.wpi.first.wpilibj.PowerDistributionPanel;

public class CurrentBreaker {

	PowerDistributionPanel pdp;
	int portnum;
	double currentThreshold;
	boolean tripped = false;
	int maxToleratedDuration;
	long trippedTime = -1;

	public CurrentBreaker(int portnum, double currentThreshold, int maxToleratedDuration) {
		pdp = new PowerDistributionPanel();
		this.portnum = portnum;
		this.currentThreshold = currentThreshold;
		this.maxToleratedDuration = maxToleratedDuration;
	}

	private void checkCurrent() {
		double current = pdp.getCurrent(portnum);

		if (current < currentThreshold) {
			trippedTime = -1;
			return;
		}

		if (current >= currentThreshold && trippedTime == -1) {
			trippedTime = System.currentTimeMillis() + maxToleratedDuration;
		}

		if (trippedTime != -1 && System.currentTimeMillis() >= trippedTime) {
			tripped = true;
		}
	}

	public boolean tripped() {
		checkCurrent();
		return tripped;
	}

	public void reset() {
		tripped = false;
		trippedTime = -1;
	}

	public double getCurrent() {
		return pdp.getCurrent(portnum);
	}
}
