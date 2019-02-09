//Left or right stating position to baseline

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoDrivePIDTune extends AutoBaseClass {
	int lastPosition = 0;
	
	public AutoDrivePIDTune(char robotPosition) {
		super(robotPosition);
	}

	public void tick() {
		
		if (isRunning()) {

			int sdPosition = (int)SmartDashboard.getNumber("Drive To Setpoint", 0);
			if (sdPosition != lastPosition && Math.abs(sdPosition) < 250) {
				DriveAuto.driveInches(sdPosition, 0, .6);
				lastPosition = sdPosition;
			}
		}
	}
}