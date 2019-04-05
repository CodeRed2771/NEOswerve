package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*
	Program that drives off platform and places a hatch panel
 */

public class AutoDriveOffPlatform extends AutoBaseClass {
	
	AutoBaseClass mSubAutoProg = new AutoDoNothing();

	public AutoDriveOffPlatform() {
		super();
	}

	public void tick() {
		if (isRunning()) {

			DriveAuto.tick();

			SmartDashboard.putNumber("Auto Step", getCurrentStep());
			switch (getCurrentStep()) {
			case 0:
				driveInches(48, 0, .8, false);
				// if (robotPosition() == Position.CENTER) {
				// 	mSubAutoProg = new AutoFindHatch();
				// 	mSubAutoProg.start();
				// } else {
				// 	// jump to steps to handle side programs
				// 	setStep(30);
				// }
				advanceStep();
				break;
			case 1:
				// mSubAutoProg.tick();
				// if (!mSubAutoProg.isRunning())
				// 	setStep(100);
				if (driveCompleted())
					advanceStep();
				
				break;
			case 2:
					stop();
					break;
				// routines for side positions
			case 30:
				driveInches(100, 0, .5);
				setTimerAndAdvanceStep(5000);
				break;
			case 31:
				if (driveCompleted())
					advanceStep();
				break;
			case 32:
				if (robotPosition() == Position.LEFT) {
					turnDegrees(-45, .35);
				} else {
					turnDegrees(45, .35);
				}
				setTimerAndAdvanceStep(2000);
				break;
			case 33:
				if (turnCompleted()) {
					advanceStep();
				}
				break;
			case 34:
				mSubAutoProg = new AutoDoEverything();
				mSubAutoProg.start();
				advanceStep();
				break;
			case 35:
				mSubAutoProg.tick();
				if (!mSubAutoProg.isRunning())
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
