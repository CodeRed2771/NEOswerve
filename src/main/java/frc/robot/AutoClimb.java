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
				break;
			case 1:
				break;
			}
		}
	}
}
