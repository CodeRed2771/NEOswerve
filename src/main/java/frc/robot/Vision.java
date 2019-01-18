/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class Vision {
    private static Vision instance;
    private static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    public static Vision getInstance() {
		if (instance == null) {
			try {
				instance = new Vision();
			} catch (Exception ex) {
				System.out.println("Vision module could not be initialized due to the following error: ");
				System.out.println(ex.getMessage());
				System.out.println(ex.getStackTrace());
				return null;
			}
		}
		return instance;
    }
    
    public static void setLED(boolean turnOn) {
        table.getEntry("ledMode").forceSetNumber(turnOn ? 3 : 1); // 3 - on, 1 = off, 2 - blink
	}
	
	public static void setDriverMode(){
		table.getEntry("camMode").forceSetNumber(1);
	}

	public static void setVisionMode(){
		table.getEntry("camMode").forceSetNumber(0);
	}

    public static void codeExample() {
        double targetOffsetAngle_Horizontal = table.getEntry("tx").getDouble(0);
		double targetOffsetAngle_Vertical = table.getEntry("ty").getDouble(0);
		double targetArea = table.getEntry("ta").getDouble(0);
		double targetSkew = table.getEntry("ts").getDouble(0);
        double targetCount = table.getEntry("tv").getDouble(0);
        
        SmartDashboard.putNumber("Target Area", targetArea);
		SmartDashboard.putNumber("Horizontal Offset Angle", targetOffsetAngle_Horizontal);
		SmartDashboard.putNumber("Vertical Offset Angle", targetOffsetAngle_Vertical);
		SmartDashboard.putNumber("target Count", targetCount);
        
        // TURN CODE FOR MAIN PROGRAM
        double turnP = .03;
		double distP = .12;
		double turnSpeed = 0;
		double fwdSpeed = 0;

		fwdSpeed = (5 - targetArea) * distP;
        turnSpeed = targetOffsetAngle_Horizontal * turnP;
        
		SmartDashboard.putNumber("fwd speed", fwdSpeed);
        SmartDashboard.putNumber("turn speed", turnSpeed);

        if (targetCount > 0) {
			
			// leftBack.set(fwdSpeed + turnSpeed);
			// leftFront.set(fwdSpeed + turnSpeed);
			// rightBack.set(-fwdSpeed + turnSpeed);
			// rightFront.set(-fwdSpeed + turnSpeed);
			
		} else {
			// leftBack.set(0);
			// leftFront.set(0);
			// rightBack.set(0);
			// rightFront.set(0);
		}
    }

}
