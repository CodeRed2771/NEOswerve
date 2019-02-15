package frc.robot;

import java.math.BigDecimal;
import java.math.RoundingMode;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveAuto {
	private static DriveAuto instance;
	private static PIDController rotDrivePID;

	private static boolean isDriving = false;
	private static boolean isTurning = false;

	private static double heading = 0; // keeps track of intended heading - used for driving "straight"
	private static double strafeAngle = 0;

	// variable to handle controlled turns
	private static double interimTurnSetpoint = 0;
	private static double finalTurnSetpoint = 0;
	private static double currentTurnSpeed = 0;
	private static double maxTurnSpeed = 1.5; // max degrees per cycle (per 20ms)
	private static double maxTurnAccel = .25; // max increase in degrees per cycle

	
	public static enum DriveSpeed {
		VERY_LOW_SPEED, LOW_SPEED, MED_SPEED, HIGH_SPEED
	};

	public static DriveAuto getInstance() {
		if (instance == null)
			instance = new DriveAuto();
		return instance;
	}

	public DriveAuto() {
		DriveTrain.getInstance();

		rotDrivePID = new PIDController(Calibration.AUTO_ROT_P, Calibration.AUTO_ROT_I, Calibration.AUTO_ROT_D,
				RobotGyro.getInstance(), rot -> DriveTrain.autoSetRot(rot));

		rotDrivePID.setAbsoluteTolerance(1.5); // degrees off
		// rotDrivePID.setToleranceBuffer(3);

		DriveTrain.setDriveMMAccel(Calibration.DT_MM_ACCEL);
		DriveTrain.setDriveMMVelocity(Calibration.DT_MM_VELOCITY);

		SmartDashboard.putNumber("AUTO DRIVE P", Calibration.AUTO_DRIVE_P);
		SmartDashboard.putNumber("AUTO DRIVE I", Calibration.AUTO_DRIVE_I);
		SmartDashboard.putNumber("AUTO DRIVE D", Calibration.AUTO_DRIVE_D);

		SmartDashboard.putNumber("TURN P", Calibration.TURN_P);
		SmartDashboard.putNumber("TURN I", Calibration.TURN_I);
		SmartDashboard.putNumber("TURN D", Calibration.TURN_D);

		SmartDashboard.putNumber("DRIVE MM VELOCITY", Calibration.DT_MM_VELOCITY);
		SmartDashboard.putNumber("DRIVE MM ACCEL", Calibration.DT_MM_ACCEL);

		SmartDashboard.putNumber("ROT P", Calibration.AUTO_ROT_P);
		SmartDashboard.putNumber("ROT I", Calibration.AUTO_ROT_I);
		SmartDashboard.putNumber("ROT D", Calibration.AUTO_ROT_D);

		SmartDashboard.putBoolean("Tune Drive/Turn PIDs", true);

		SmartDashboard.putNumber("ROT Max Deg/Cycle", maxTurnSpeed);
		SmartDashboard.putNumber("ROT Max Acc/Cycle", maxTurnAccel);
	
	}

	public static void driveInches(double inches, double angle, double speedFactor) {

		SmartDashboard.putNumber("DRIVE INCHES", inches);

		strafeAngle = angle;

		stopTurning();

		isDriving = true;

		DriveTrain.setDriveMMVelocity((int) (Calibration.DT_MM_VELOCITY * speedFactor));

		rotDrivePID.disable();

		// angle at which the wheel modules should be turned
		DriveTrain.setAllTurnOrientiation(-DriveTrain.angleToLoc(strafeAngle));

		// give it just a little time to get the modules turned to position
		// before starting the drive
		// this helps to get accurate encoder readings too since the drive
		// encoder values are affected by turning the modules
		try {
			Thread.sleep(150);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		// set the new drive distance setpoint
		DriveTrain.addToAllDrivePositions(convertToTicks(inches));
	}

	public static void reset() {
		stop();
		DriveTrain.resetDriveEncoders();
		rotDrivePID.reset();
		rotDrivePID.setSetpoint(0);
		heading = RobotGyro.getAngle();
		isTurning = false;
		isDriving = false;
	}

	public static void stop() {
		stopTurning();
		stopDriving();
	}

	public static void stopDriving() {
		isDriving = false;
		DriveTrain.stopDriveAndTurnMotors();
	}

	public static void stopTurning() {
		rotDrivePID.setSetpoint(rotDrivePID.get());
		rotDrivePID.disable();
		isTurning = false;
		DriveTrain.stopDriveAndTurnMotors();
	}

	public static void setTurnDegreesToCurrentAngle() {
		// this is necessary so that subsequent turns are relative to the current
		// position. Otherwise they'd always be relative to 0
		rotDrivePID.setSetpoint(RobotGyro.getAngle());
	}

	public static void turnDegrees(double degrees, double turnSpeedFactor) {
		// Turns using the Gyro, relative to the current position
		// Use "turnCompleted" method to determine when the turn is done
		// The PID controller for this sends a rotational value to the
		// standard swerve drive method to make the bot rotate

		isDriving = false;
		heading += degrees; // this is used later to help us drive straight
							// after rotating

		SmartDashboard.putNumber("TURN DEGREES CALL", degrees);
		SmartDashboard.putNumber("ROT SETPOINT", rotDrivePID.getSetpoint() + degrees);

		rotDrivePID.setSetpoint(rotDrivePID.getSetpoint() + degrees);
		rotDrivePID.enable();
		setRotationalPowerOutput(turnSpeedFactor);

		try {
			Thread.sleep(100);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	public static void turnDegreesV2(double degrees, double turnSpeedFactor) {
		// Turns using the Gyro, relative to the current position
		// Use "turnCompleted" method to determine when the turn is done
		// The PID controller for this sends a rotational value to the
		// standard swerve drive method to make the bot rotate

		stopDriving();

		isTurning = true;

		interimTurnSetpoint = heading; // should be our current position
		heading += degrees; // this is used later to help us drive straight
							// after rotating
		finalTurnSetpoint = heading;
		currentTurnSpeed = 0;

		SmartDashboard.putNumber("TURN DEGREES CALL", degrees);
		SmartDashboard.putNumber("ROT SETPOINT", finalTurnSetpoint);

		setRotationalPowerOutput(turnSpeedFactor);
		rotDrivePID.setSetpoint(interimTurnSetpoint);

		rotDrivePID.enable(); // the setpoint will be updated in the tick() method
	}

	// public static void continuousTurn(double degrees, double maxPower) {
	// motionStartTime = System.currentTimeMillis();
	//
	// rotDrivePID.setSetpoint(RobotGyro.getAngle() + degrees);
	// rotDrivePID.enable();
	// setRotationalPowerOutput(maxPower);
	// }
	//
	public static void continuousDrive(double inches, double maxPower) {
		isTurning = false;
		setRotationalPowerOutput(maxPower);

		DriveTrain.setTurnOrientation(DriveTrain.angleToLoc(0), DriveTrain.angleToLoc(0), DriveTrain.angleToLoc(0),
				DriveTrain.angleToLoc(0));
		rotDrivePID.disable();
	}

	public static void tick() {
		// this is called roughly 50 times per second

		maxTurnSpeed = SmartDashboard.getNumber("ROT Max Deg/Cycle", maxTurnSpeed);
		maxTurnAccel = SmartDashboard.getNumber("ROT Max Acc/Cycle", maxTurnAccel);

		if (isTurning) {
			double cyclesToDecelerate = maxTurnSpeed / maxTurnAccel;
			double cyclesLeft = Math.abs(finalTurnSetpoint - interimTurnSetpoint) / maxTurnSpeed;
			if (cyclesLeft > cyclesToDecelerate) {
				// accelerate
				if (currentTurnSpeed < maxTurnSpeed) {
					currentTurnSpeed += maxTurnAccel;
					if (currentTurnSpeed > maxTurnSpeed)
						currentTurnSpeed = maxTurnSpeed;
				}
			} else {
				// decelerate, but no slower than the acceleration factor
				if (currentTurnSpeed > maxTurnAccel) {
					currentTurnSpeed -= maxTurnAccel;
					if (currentTurnSpeed < maxTurnAccel)
						currentTurnSpeed = maxTurnAccel;
				}
			}
			
			if (interimTurnSetpoint < finalTurnSetpoint) {
				// we're increasing towards the final point

				interimTurnSetpoint += currentTurnSpeed;

				// if we set the interim past the final, adjust it to final
				if (interimTurnSetpoint > finalTurnSetpoint)
					interimTurnSetpoint = finalTurnSetpoint;
			} else {
				// we're reversing towards the final point
				interimTurnSetpoint -= currentTurnSpeed;

				// if we set the interim past the final, adjust it to final
				if (interimTurnSetpoint < finalTurnSetpoint)
					interimTurnSetpoint = finalTurnSetpoint;
			}

			rotDrivePID.setSetpoint(interimTurnSetpoint);
		}

		SmartDashboard.putNumber("ROT PID ERROR", rotDrivePID.getError());
		SmartDashboard.putNumber("Drive Train Velocity", DriveTrain.getDriveVelocity());
		SmartDashboard.putBoolean("HasArrived", hasArrived());
		SmartDashboard.putBoolean("TurnCompleted", turnCompleted());
		SmartDashboard.putNumber("Drive PID Error", DriveTrain.getDriveError());

		// Sets the PID values based on input from the SmartDashboard
		// This is only needed during tuning
		if (SmartDashboard.getBoolean("Tune Drive/Turn PIDs", false)) {
			rotDrivePID.setPID(SmartDashboard.getNumber("ROT P", Calibration.AUTO_ROT_P),
					SmartDashboard.getNumber("ROT I", Calibration.AUTO_ROT_I),
					SmartDashboard.getNumber("ROT D", Calibration.AUTO_ROT_D));

			DriveTrain.setDrivePIDValues(SmartDashboard.getNumber("AUTO DRIVE P", Calibration.AUTO_DRIVE_P),
					SmartDashboard.getNumber("AUTO DRIVE I", Calibration.AUTO_DRIVE_I),
					SmartDashboard.getNumber("AUTO DRIVE D", Calibration.AUTO_DRIVE_D));
			//
			DriveTrain.setTurnPIDValues(SmartDashboard.getNumber("TURN P", Calibration.TURN_P),
					SmartDashboard.getNumber("TURN I", Calibration.TURN_I),
					SmartDashboard.getNumber("TURN D", Calibration.TURN_D));

			DriveTrain.setDriveMMAccel((int) SmartDashboard.getNumber("DRIVE MM ACCEL", Calibration.DT_MM_ACCEL));
			DriveTrain.setDriveMMVelocity(
					(int) SmartDashboard.getNumber("DRIVE MM VELOCITY", Calibration.DT_MM_VELOCITY));
		}
		// Use the gyro to try to drive straight. This only works if we're not
		// strafing because it turns the modules a little bit to keep the robot
		// straight.
		// The "heading" reflects any rotation we may have done to the
		// "straight" driving will be at the angle that the robot has been
		// rotated to.

		// if (strafeAngle == 0) { // currently this routine only works when

		// if (Math.abs(strafeAngle) < 60) { // not effective for high strafe
		// // angles
		// if (isDriving) {
		// // this gets a -180 to 180 value i believe
		// double rawGyroPidGet = RobotGyro.getGyro().pidGet();

		// double adjust = (rawGyroPidGet - heading) * .5;

		// // THIS IS THE GYRO CORRECTION I WANT TO TRY
		// if (DriveTrain.getDriveVelocity() > 0) // driving forward or
		// // backward
		// DriveTrain.setTurnOrientation(DriveTrain.angleToLoc(-strafeAngle + adjust),
		// DriveTrain.angleToLoc(-strafeAngle - adjust),
		// DriveTrain.angleToLoc(-strafeAngle - adjust),
		// DriveTrain.angleToLoc(-strafeAngle + adjust));
		// else
		// DriveTrain.setTurnOrientation(DriveTrain.angleToLoc(-strafeAngle - adjust),
		// DriveTrain.angleToLoc((-strafeAngle + adjust)),
		// DriveTrain.angleToLoc((-strafeAngle + adjust)),
		// DriveTrain.angleToLoc(-strafeAngle - adjust));

		// SmartDashboard.putNumber("Angle Adjustment", adjust);
		// SmartDashboard.putNumber("Adjusted Angle", strafeAngle - adjust);
		// // ORIGINAL
		// // Also include the strafeAngle == 0

		// // if (DriveTrain.getDriveError() > 0) // directional difference
		// // DriveTrain.setTurnOrientation(DriveTrain.angleToLoc(adjust),
		// // DriveTrain.angleToLoc(-adjust),
		// // DriveTrain.angleToLoc(-adjust),
		// // DriveTrain.angleToLoc(adjust));
		// // else
		// // DriveTrain.setTurnOrientation(DriveTrain.angleToLoc(-adjust),
		// // DriveTrain.angleToLoc(adjust),
		// // DriveTrain.angleToLoc(adjust),
		// // DriveTrain.angleToLoc(-adjust));
		// }
		// }

	}

	private static void setRotationalPowerOutput(double powerLevel) {
		rotDrivePID.setOutputRange(-powerLevel, powerLevel);
	}

	public static double getDistanceTravelled() {
		return Math.abs(convertTicksToInches(DriveTrain.getDriveEnc()));
	}

	public static boolean hasArrived() {
		return DriveTrain.hasDriveCompleted(10);
	}

	public static boolean turnCompleted(double allowedError) {

		// TO DO - DVV - we should try changing to this:
		// return rotDrivePID.onTarget();

		return Math.abs(RobotGyro.getAngle() - heading) <= allowedError;
	}

	public static boolean turnCompleted() {
		return turnCompleted(1); // allow 1 degree of error by default
	}

	public static void setPIDstate(boolean isEnabled) {
		if (isEnabled) {
			rotDrivePID.enable();
		} else {
			rotDrivePID.disable();
		}
	}

	public static void disable() {
		setPIDstate(false);
	}

	private static int convertToTicks(double inches) {
		return (int) (inches * Calibration.DRIVE_DISTANCE_TICKS_PER_INCH);
	}

	private static double convertTicksToInches(int ticks) {
		return ticks / Calibration.DRIVE_DISTANCE_TICKS_PER_INCH;
	}

	public static void showEncoderValues() {
		SmartDashboard.putNumber("Drive Encoder", DriveTrain.getDriveEnc());

		SmartDashboard.putNumber("Drive PID Error", DriveTrain.getDriveError());
		SmartDashboard.putNumber("Drive Avg Error", DriveTrain.getAverageDriveError());

		SmartDashboard.putNumber("Gyro", round2(RobotGyro.getAngle()));
		// SmartDashboard.putNumber("Gyro PID Setpoint",
		// rotDrivePID.getSetpoint());
		// SmartDashboard.putNumber("Gyro PID error",
		// round2(rotDrivePID.getError()));

		// SmartDashboard.putBoolean("Has Arrived", hasArrived());

		// SmartDashboard.putNumber("Left Drive Encoder Raw: ",
		// -mainDrive.getLeftEncoderObject().get());
		// SmartDashboard.putNumber("Right Drive Encoder Raw: ",
		// -mainDrive.getRightEncoderObject().get());

		// SmartDashboard.putNumber("Right PID error",
		// rightDrivePID.getError());
		// SmartDashboard.putNumber("Left Drive Encoder Get: ",
		// mainDrive.getLeftEncoderObject().get());
		// SmartDashboard.putNumber("Right Drive Encoder Get: ",
		// mainDrive.getRightEncoderObject().get());
		// SmartDashboard.putNumber("Left Drive Distance: ",
		// leftEncoder.getDistance());
		// SmartDashboard.putNumber("Right Drive Distance: ",
		// rightEncoder.getDistance());
		// SmartDashboard.putNumber("Right Drive Encoder Raw: ",
		// DriveTrain.getDriveEnc());
		// SmartDashboard.putNumber("Right Setpoint: ",
		// rightDrivePID.getSetpoint());

	}

	private static Double round2(Double val) {
		// added this back in on 1/15/18
		return new BigDecimal(val.toString()).setScale(2, RoundingMode.HALF_UP).doubleValue();
	}
}
