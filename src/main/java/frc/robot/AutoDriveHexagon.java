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
				driveInches(20, 0, .8, false);
				setTimerAndAdvanceStep(2000);
				break;
			case 1:
				if (driveCompleted())
					advanceStep();
				break;
			case 2:
				driveInches(20, 60, .8, false);
				setTimerAndAdvanceStep(2000);
				break;
			case 3:
				if (driveCompleted())
					advanceStep();
				break;
			case 4:
				driveInches(20, 120, .8, false);
				setTimerAndAdvanceStep(2000);
				break;
			case 5:
				if (driveCompleted())
					advanceStep();
				break;
			case 6:
				driveInches(20, 180, .8, false);
				setTimerAndAdvanceStep(2000);
				break;
			case 7:
				if (driveCompleted())
					advanceStep();
				break;
			case 8:
				driveInches(20, 240, .8, false);
				setTimerAndAdvanceStep(2000);
				break;
			case 9:
				if (driveCompleted())
					advanceStep();
				break;
			case 10:
				driveInches(20, 300, .8, false);
				setTimerAndAdvanceStep(2000);
				break;
			case 11:
				if (driveCompleted())
					advanceStep();
				break;
			case 12:
				setStep(100);
				break;
			// keep this case as the final case, and keep it 100
			case 100:
				stop();
				break;
			}
		}
	}
}
