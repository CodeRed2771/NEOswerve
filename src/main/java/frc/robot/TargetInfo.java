/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Add your docs here.
 */
public class TargetInfo {
    
    private static TargetInfo instance;

    public static TargetInfo getInstance() {
		if (instance == null) {
			try {
				instance = new TargetInfo();
			} catch (Exception ex) {
				System.out.println("Target Info module could not be initialized due to the following error: ");
				System.out.println(ex.getMessage());
				System.out.println(ex.getStackTrace());
				return null;
			}
		}
		return instance;
    }
    
    public static double targetAngle() {
        double currentGyroAngle = RobotGyro.getAngle();

        if (currentGyroAngle > 6 && currentGyroAngle < 75) {
            return 30;
        }
        if (currentGyroAngle > 76 && currentGyroAngle < 170) {
            return 90;
        }
        if (currentGyroAngle < -10 && currentGyroAngle > -90) {
            return 150;
        }
        if (currentGyroAngle < 5 && currentGyroAngle > -5) {
            return 0;
        }

        // NEED MORE IF STATEMENTS FOR OTHER POSITIONS
        
        return 90;
        
    }
}

