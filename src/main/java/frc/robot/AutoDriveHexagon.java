package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*
 */

public class AutoDriveHexagon extends AutoBaseClass {

	AutoBaseClass mSubAutoProg = new AutoDoNothing();

	public AutoDriveHexagon() {
		super();
	}

	public void tick() {
		if (isRunning()) {

			DriveAuto.tick();

			SmartDashboard.putNumber("Auto Step", getCurrentStep());
			switch (getCurrentStep()) {
			case 0:
				driveInches(20, 0, .5, true);
				setTimerAndAdvanceStep(2000);
				break;
			case 1:
				if (driveCompleted())
					advanceStep();
				break;
			case 2:
				driveInches(20, 60, .5, true);
				setTimerAndAdvanceStep(2000);
				break;
			case 3:
				if (driveCompleted())
					advanceStep();
				break;
			case 4:
				driveInches(20, 120, .5, true);
				setTimerAndAdvanceStep(2000);
				break;
			case 5:
				if (driveCompleted())
					advanceStep();
				break;
			case 6:
				driveInches(20, 180, .5, true);
				setTimerAndAdvanceStep(2000);
				break;
			case 7:
				if (driveCompleted())
					advanceStep();
				break;
			case 8:
				driveInches(20, 240, .5, true);
				setTimerAndAdvanceStep(2000);
				break;
			case 9:
				if (driveCompleted())
					advanceStep();
				break;
			case 10:
				driveInches(20, 300, .5, true);
				setTimerAndAdvanceStep(2000);
				break;
			case 11:
				if (driveCompleted())
					advanceStep();
				break;
			// keep this case as the final case, and keep it 100
			case 100:
				stop();
				break;
			}
		}
	}
}
