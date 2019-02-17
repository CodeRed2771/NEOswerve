package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*
 Slides over one cargo opening on the cargo ship.
 */

public class AutoSlideOver extends AutoBaseClass {
	public AutoSlideOver() {
		super();
	}

	public void tick() {
		SmartDashboard.putNumber("Auto Step", getCurrentStep());
		if (isRunning()) {

			switch (getCurrentStep()) {
			case 0:
				setTimerAndAdvanceStep(2000);
				if (slideDirection() == Direction.LEFT) {
					driveInches(-13, 70, .2);
				} else {
					driveInches(-13, -70, .2);
				}
				break;
			case 1:
				if (driveCompleted())
					advanceStep();
				break;
			case 2:
				setTimerAndAdvanceStep(2000);
				if (slideDirection() == Direction.LEFT) {
					driveInches(13, -70, .2);
				} else {
					driveInches(13, 70, .2);
				}
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
