/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Vision
 * 
 * To track a vision target
 * Call setVisionTrackingMode() to entable tracking
 * Call "Vision.tick()" in your periodic loop to continually check for a target
 * and store the horizontal offset for when you ask for it.  This avoids the issue
 * where the image happens not to be valid the instant you check it.  With this 
 * code, if it was valid up to a half second earlier, it uses those values.
 * Your code should check targetInfoIsValid() before bothering to read the data.
 * If the targetInfo is not valid, the data function will return a 0.
 * 
 */
public class Vision implements PIDSource {
    private static Vision instance;
    private static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
	private static long lastValidReadTime; 
	private static double storedOffsetFromTarget = 0;
	private static double storedTargetArea = 0;
	private static double storedTargetSkew = 0;

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
		lastValidReadTime = System.currentTimeMillis() - 5000;  // 5 seconds back in time
	}

	public static void tick() {
		readTarget();
	}
	
	public static void readTarget() {
		if (inVisionTrackingMode() && targetCount() > 0) {
			lastValidReadTime = System.currentTimeMillis();
			storedOffsetFromTarget = table.getEntry("tx").getDouble(0);
			storedTargetArea = table.getEntry("ta").getDouble(0);
		}
	}

    public static void setLED(boolean turnOn) {
        table.getEntry("ledMode").forceSetNumber(turnOn ? 3 : 1); // 3 - on, 1 = off, 2 - blink
	}
	
	public static boolean inVisionTrackingMode() {
		return (table.getEntry("camMode").getNumber(0).intValue() == 0);
	}

	public static void setDriverMode(){
		table.getEntry("camMode").forceSetNumber(1);
	}

	public static void setVisionTrackingMode(){
		table.getEntry("camMode").forceSetNumber(0);
	}

	public static boolean targetInfoIsValid() {
		readTarget();
		return  (System.currentTimeMillis() - lastValidReadTime) < 500;  // less than 500 ms old
	}

	public static double distance(){
		// Min distance = 3ft 5in
			// Target area at 4ft - .82
			// Target area at 5ft - .7
			// Target area at 6ft - .48
			// Target area at 7ft - .3
			// Target area at 8ft - .1
		// Max distance = 13ft 4in

		return targetArea();
	}

	public static double offsetFromTarget() {
		readTarget();
		if (targetInfoIsValid()) {
			return storedOffsetFromTarget;
		} else
			return 0;
	}

	public static double targetArea() {
		readTarget();
		if (targetInfoIsValid()) {
			return storedTargetArea;
		} else {
			return 0;
		}
	}

	public static double targetSkew() {
		readTarget();
		if (targetInfoIsValid()) {
			return storedTargetSkew;
		} else
			return 0;
	}

	public static int targetCount() {
		return (int)table.getEntry("tv").getDouble(0);
	}

	public PIDSourceType getPIDSourceType() {
		return PIDSourceType.kDisplacement;
	}
	public void setPIDSourceType(PIDSourceType pType ) {

	}
	public double pidGet() {
		return -offsetFromTarget();
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
