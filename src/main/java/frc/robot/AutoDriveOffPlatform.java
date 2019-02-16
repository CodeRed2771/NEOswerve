package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*
 Slides over one cargo opening on the cargo ship.
 */

public class AutoDriveOffPlatform extends AutoBaseClass {
	char robotPosition;

	public AutoDriveOffPlatform() {
		super();

	}

	public void start(char robotPosition) {
		this.robotPosition = robotPosition;
		super.start();
	}

	public void tick() {
		if (isRunning()) {

			SmartDashboard.putNumber("Auto Step", getCurrentStep());

			switch (getCurrentStep()) {
			case 0:
				setTimerAndAdvanceStep(5000);
				break;
			case 1:
				if (driveCompleted())
					advanceStep();
				break;
			case 2:
				setTimerAndAdvanceStep(2000);
				break;
			case 3:
				if (driveCompleted())
					advanceStep();
				break;
			case 4:
				stop();
				break;

			}
		}
	}
}
