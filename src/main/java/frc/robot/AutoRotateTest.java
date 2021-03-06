package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*
 * This auto program is a rotation
 * calibration test.
 */

public class AutoRotateTest extends AutoBaseClass {
	
	public void tick() {
		if (isRunning()) {

			SmartDashboard.putNumber("Auto Step", getCurrentStep());

			switch (getCurrentStep()) {
			case 0:
				setTimerAndAdvanceStep(4000);
				turnToHeading(90, SmartDashboard.getNumber("Turn Speed", 0.25));
				break;
			case 1:
				if (turnCompleted())
					advanceStep();
				break;
			case 2:
				setTimerAndAdvanceStep(5000);
				break;
			case 3:
				break;
			case 4:
				setTimerAndAdvanceStep(4000);
				turnToHeading(270, SmartDashboard.getNumber("Turn Speed", 0.4));
				break;
			case 5:
				if (turnCompleted())
					advanceStep();
				break;
			case 6:
				setTimerAndAdvanceStep(5000);
				break;
			case 7:
				break;
			case 8: 
				setStep(0);
				break;
			// case 6:
			// setTimerAndAdvanceStep(1000);
			// break;
			// case 7:
			// break;
			// case 8:
			// setTimerAndAdvanceStep(4000);
			// driveInches(0, 0, .3);
			// break;
			// case 9:
			// if(driveCompleted())
			// advanceStep();
			// break;
			case 10:
				setStep(100000);
				// DriveTrain.setTurnOrientation(0, 0, 0, 0);
			}
		}
	}
}
