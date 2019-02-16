package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*
 Slides over one cargo opening on the cargo ship.
 */

public class AutoSlideOver extends AutoBaseClass {
	Direction direction;

	public static enum Direction {
		LEFT, RIGHT
	};

	public AutoSlideOver() {
		super();
		
	}

	public void start (Direction direction) {
		this.direction = direction;
		super.start();
	}

	public void tick() {
		if (isRunning()) {

			SmartDashboard.putNumber("Auto Step", getCurrentStep());

			switch (getCurrentStep()) {
			case 0:
				setTimerAndAdvanceStep(2000);
				if (direction == Direction.LEFT) {
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
				if (direction == Direction.LEFT) {
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
