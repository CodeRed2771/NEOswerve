package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*
	Program that climbs onto HAB
 */

public class AutoClimb extends AutoBaseClass {
    // Run climb motor to full height position
    // Rotate rear drive modules (B & C) to 90 degrees
    // Engage B module to drive forward a certain distance
    // Run climb motor backwards to start position
    
	public AutoClimb() {
		super();
	}

	public void tick() {
		if (isRunning()) {

			DriveAuto.tick();

			SmartDashboard.putNumber("Auto Step", getCurrentStep());
			switch (getCurrentStep()) {
			case 0:
				Climber.climberExtend();
				break;
			case 1:
				if (Climber.isExtended()) {
					setStep(2);
				}
				break;
			case 2:
				DriveTrain.moduleB.setTurnOrientation(.25);
				DriveTrain.moduleC.setTurnOrientation(.25);
				break;
			case 3:
				if (DriveTrain.moduleB.hasDriveCompleted(20)) {
					DriveAuto.stopDriving();
					setStep(4);
				}
				break;
			case 4:
				Climber.climberRetract();
				setTimerAndAdvanceStep(5000);
				break;
			case 5:
				break;
			case 6:
				stop();
				break;
			}
		}
	}
}
