package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*
	Program that climbs onto HAB
 */

public class AutoClimb extends AutoBaseClass {
    // Run climb motor to full height position
    // Rotate rear drive modules (B & C) to 90 degrees

    
	public AutoClimb() {
		super();
	}

	public void tick() {
		if (isRunning()) {

			DriveAuto.tick();

			SmartDashboard.putNumber("Auto Step", getCurrentStep());
			switch (getCurrentStep()) {
			case 0:
				Manipulator.linkageUp();
				Climber.climberExtend();
				Climber.setClimbDriveSpeed(-.25);
				setTimerAndAdvanceStep(5000);
				break;
			case 1:
				if (Climber.isExtended()) {
					setStep(3);
				}
				break;
			case 2:
				// if we get here we failed to climb
				setStep(99);
				break;
			case 3:
				DriveTrain.moduleB.setTurnOrientation(.25);
				DriveTrain.moduleC.setTurnOrientation(.25);
				Climber.setClimbDriveSpeed(-.6); // drive forward
				setTimerAndAdvanceStep(4000);
				break;
			case 4:
				break;
			case 5:
				setTimerAndAdvanceStep(250);
				Climber.setClimbDriveSpeed(.6); // drive back a titch
				break;
			case 6:
				break;
			case 7:
				Climber.setClimbDriveSpeed(0);
				Climber.climberRetract();
				setTimerAndAdvanceStep(3000);
				break;
			case 8:
				break;
			case 9:
				setStep(99);
				break;
			case 99:
				Climber.stop();
				stop();
				break;
			}
		}
	}
}
