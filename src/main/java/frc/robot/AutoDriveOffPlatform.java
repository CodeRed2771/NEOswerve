package frc.robot;

import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*
 Slides over one cargo opening on the cargo ship.
 */

public class AutoDriveOffPlatform extends AutoBaseClass {
	
	AutoBaseClass mSubAutoProg = new AutoDoNothing();

	public AutoDriveOffPlatform() {
		super();
	}

	public void tick() {
		if (isRunning()) {
			SmartDashboard.putNumber("Auto Step", getCurrentStep());
			switch (getCurrentStep()) {
			case 0:
				if (robotPosition() == Position.CENTER) {
					mSubAutoProg = new AutoFindHatch();
					mSubAutoProg.start();
				} else {
					// jump to steps to handle side programs
					setStep(30);
				}
				advanceStep();
				break;
			case 1:
				mSubAutoProg.tick();
				if (!mSubAutoProg.isRunning())
					setStep(100);
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
				mSubAutoProg = new AutoFindHatch();
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
