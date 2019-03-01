/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// Min distance = 3ft 5in
// Target area at 4ft - .82
// Target area at 5ft - .7
// Target area at 6ft - .48
// Target area at 7ft - .3
// Target area at 8ft - .1
// Max distance = 13ft 4in

package frc.robot;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Vision
 * 
 * To track a vision target Call setVisionTrackingMode() to entable tracking
 * Call "Vision.tick()" in your periodic loop to continually check for a target
 * and store the horizontal offset for when you ask for it. This avoids the
 * issue where the image happens not to be valid the instant you check it. With
 * this code, if it was valid up to a half second earlier, it uses those values.
 * Your code should check targetInfoIsValid() before bothering to read the data.
 * If the targetInfo is not valid, the data function will return a 0.
 * 
 */
public class Vision {
	private static Vision instance;
	private static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
	private static long lastValidReadTime;
	private static double storedOffsetFromTarget = 0;
	private static double storedTargetArea = 0;
	private static double storedTargetSkew = 0;
	private static double dis = 0;
	private static int validCount = 0;

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

	public Vision() {
		// set the last valid read time to an old time to make it invalid
		lastValidReadTime = System.currentTimeMillis() - 5000; // 5 seconds back in time
	}

	public static void tick() {
		readTargetInfo();
	}

	public static void readTargetInfo() {
		validCount++;
		if (inVisionTrackingMode() && targetCount() > 0 && validCount > 5) {

			lastValidReadTime = System.currentTimeMillis();

			storedOffsetFromTarget = table.getEntry("tx").getDouble(0);
			storedTargetArea = table.getEntry("ta").getDouble(0);
			storedTargetSkew = table.getEntry("ts").getDouble(0);
		} else {
			if (targetCount() == 0)
				validCount = 0;
		}
	}

	public static void setLED(boolean turnOn) {
		table.getEntry("ledMode").forceSetNumber(turnOn ? 3 : 1); // 3 - on, 1 = off, 2 - blink
	}

	public static boolean inVisionTrackingMode() {
		return (table.getEntry("camMode").getNumber(0).intValue() == 0);
	}

	public static void setDriverMode() {
		setLED(false);
		table.getEntry("camMode").forceSetNumber(1);
	}

	private static void setVisionTrackingMode() {
		table.getEntry("camMode").forceSetNumber(0);
	}

	public static boolean targetInfoIsValid() {
		readTargetInfo();
		return (System.currentTimeMillis() - lastValidReadTime) < 500; // less than 500 ms old
	}

	public static double offsetFromTarget() {
		readTargetInfo();
		if (targetInfoIsValid()) {
			return storedOffsetFromTarget;
		} else
			return 0;
	}

	public static double targetArea() {
		readTargetInfo();
		if (targetInfoIsValid()) {
			return storedTargetArea;
		} else {
			return 0;
		}
	}

	public static double getDistanceFromTarget() {

		// Limelight 1
		// dis = (-8.1435 * targetArea() + 9.657)*12;

		// Test - to be removed
		// dis = (7.0754 * targetArea() + 9.4501)*1;

		// Limelight 2
		// dis = (-7.5541 * targetArea() + 10.367)*12;

		// Limelight 2 update 2/12/19
		// dis = (-0.6944 * targetArea() + 8.6644)*12;

		double ta = targetArea();
		double tvert = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tvert").getDouble(0);
		SmartDashboard.putNumber("tvert", tvert);

		if (tvert == 0) {
			return 0;
		} 

		return 0;

		//THIS IS HOW WE CALCULATE DISTANCE THROUGH THE TARGET AREA.

		// if (ta == 0)
		// 	return 0;
		// if (ta > 12.3)
		// 	return 24;
		// else if (ta > 8.5)
		// 	return 30;
		// else if (ta > 5.8)
		// 	return 36; 
		// else if (ta > 4.425)
		// 	return 42;
		// else if (ta > 3.05)
		// 	return 48;
		// else if (ta > 2.4785)
		// 	return 54;
		// else if (ta > 1.907)
		// 	return 60;
		// else if (ta > 1.616)
		// 	return 66;
		// else if (ta > 1.325)
		// 	return 72;
		// else if (ta > 1.1545 )
		// 	return 78;
		// else if (ta > .984)
		// 	return 84;
		// else if (ta > 0.8585) 
		// 	return 90;
		// else if (ta > .733) 
		// 	return 96;
		// else if (ta > 0.6325)
		// 	return 102;
		// else if (ta > .532) 
		// 	return 108;
		// else if (ta > 0.4835)
		// 	return 114;
		// else if (ta > .435) 
		// 	return 120;
		// else if (ta > 0.393)
		// 	return 126;
		// else if (ta > .351) 
		// 	return 132;
		// else
		// 	return 144;

		
	}

	public static double getTargetSkew() {
		readTargetInfo();
		if (targetInfoIsValid()) 
			return storedTargetSkew;
		else
			return 0;
	}

	public static int targetCount() {
		return (int) table.getEntry("tv").getDouble(0);
	}

	public static double tx() {
		if (targetCount() > 0) {
			return (double) table.getEntry("tx").getDouble(0);
		} else {
			return 0;
		}
	}

	public static void setTargetTrackingMode() {
		setLED(true);
		setVisionTrackingMode();
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
