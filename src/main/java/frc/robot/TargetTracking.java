package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TargetTracking extends AutoBaseClass {
    public TargetTracking(char robotPosition) {
		super(robotPosition);
		System.out.println("TargetTracking started");
	}
    public void tick() {
		if (isRunning()) {
			SmartDashboard.putNumber("Auto Step", getCurrentStep());

			switch (getCurrentStep()) {
			case 0: // DRIVE TO SCALE
				setTimerAndAdvanceStep(2000);
                this.driveInches(12, 0, 1);
				break;
			case 1:
				if (driveCompleted())
					advanceStep();
				break;
			case 2: // DRIVE TO SCALE PT. 2
                stop();
                break;
			}
        }
    }
}