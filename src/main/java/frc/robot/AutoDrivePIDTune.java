//Left or right stating position to baseline

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoDrivePIDTune extends AutoBaseClass {
	int lastPosition = 0;

	public void tick() {
		
		if (isRunning()) {

			int sdPosition = (int)SmartDashboard.getNumber("Drive To Setpoint", 0);
			int sdStrafeAngle = (int)SmartDashboard.getNumber("Drive Strafe Angle", 0);
			if (sdPosition != lastPosition && Math.abs(sdPosition) < 250) {
				DriveAuto.driveInches(sdPosition, sdStrafeAngle, .4);
				lastPosition = sdPosition;
			}
		}
	}
}