package frc.robot;

import java.math.BigDecimal;
import java.math.RoundingMode;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.libs.CurrentBreaker;

public class DriveAuto {
	private static DriveAuto instance;
	private static PIDController rotDrivePID;

	private static boolean isDriving = false;
	private static boolean followingTarget = false;

	private static double heading = 0; // keeps track of intended heading - used for driving "straight"
	private static double strafeAngle = 0;
	private static double strafeAngleOriginal = 0;

	private static CurrentBreaker driveCurrentBreaker;

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
				Calibration.AUTO_ROT_F, RobotGyro.getInstance(), rot -> DriveTrain.autoSetRot(rot));

		rotDrivePID.setAbsoluteTolerance(2); // degrees off
		// rotDrivePID.setToleranceBuffer(3);

		DriveTrain.setDriveMMAccel(Calibration.DT_MM_ACCEL);
		DriveTrain.setDriveMMVelocity(Calibration.DT_MM_VELOCITY);

		driveCurrentBreaker = new CurrentBreaker(Wiring.DRIVE_PDP_PORT, 55, 400); //Changed from 35 on 4/26/2019 by CS and AR
		//changed amps back to 35 on 4/27/19 because Mr. Scott said to :( ~CM
		//changed amps back to 55 on 4/27/19 at 9:20 because Mr. Scott decided that it was fine XD ~CS 
		//Yay ~CM
		driveCurrentBreaker.reset();

		// SmartDashboard.putNumber("AUTO DRIVE P", Calibration.AUTO_DRIVE_P);
		// SmartDashboard.putNumber("AUTO DRIVE I", Calibration.AUTO_DRIVE_I);
		// SmartDashboard.putNumber("AUTO DRIVE D", Calibration.AUTO_DRIVE_D);

		// SmartDashboard.putNumber("TURN P", Calibration.TURN_P);
		// SmartDashboard.putNumber("TURN I", Calibration.TURN_I);
		// SmartDashboard.putNumber("TURN D", Calibration.TURN_D);

		// SmartDashboard.putNumber("DRIVE MM VELOCITY", Calibration.DT_MM_VELOCITY);
		// SmartDashboard.putNumber("DRIVE MM ACCEL", Calibration.DT_MM_ACCEL);

		// SmartDashboard.putNumber("ROT P", Calibration.AUTO_ROT_P);
		// SmartDashboard.putNumber("ROT I", Calibration.AUTO_ROT_I);
		// SmartDashboard.putNumber("ROT D", Calibration.AUTO_ROT_D);
		// SmartDashboard.putNumber("ROT F", Calibration.AUTO_ROT_F);

		// SmartDashboard.putBoolean("Tune Drive/Turn PIDs", true);

		// SmartDashboard.putNumber("ROT Max Deg/Cycle", maxTurnSpeed);
		// SmartDashboard.putNumber("ROT Max Acc/Cycle", maxTurnAccel);

	}

	public static void driveInches(double inches, double angle, double speedFactor, boolean followTarget) {

		followingTarget = followTarget;

		SmartDashboard.putNumber("DRIVE INCHES", inches);

		strafeAngle = -angle;
		strafeAngleOriginal = strafeAngle;

		stopTurning();

		isDriving = true;

		DriveTrain.setDriveMMVelocity((int) (Calibration.DT_MM_VELOCITY * speedFactor));

		// angle at which the wheel modules should be turned

		// didnt help - DriveTrain.unReverseModules(); // make sure all "reversed" flags
		// are reset.
		DriveTrain.setAllTurnOrientation(DriveTrain.angleToPosition(strafeAngle), true);

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

	public static void driveInches(double inches, double angle, double speedFactor) {
		driveInches(inches, angle, speedFactor, false);
	}

	public static void reset() {
		stop();
		DriveTrain.resetDriveEncoders();
		rotDrivePID.reset();
		rotDrivePID.setSetpoint(0);
		heading = RobotGyro.getRelativeAngle();
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
		DriveTrain.stopDriveAndTurnMotors();
	}

	public static void setTurnDegreesToCurrentAngle() {
		// this is necessary so that subsequent turns are relative to the current
		// position. Otherwise they'd always be relative to 0
		rotDrivePID.setSetpoint(RobotGyro.getAngle());
	}

	public static double degreesToInches(double degrees) {
		double inches = degrees / 3.47;
		return inches;
	}

	public static void turnToHeading(double desiredHeading, double turnSpeedFactor) {
		double turnAmount = desiredHeading - RobotGyro.getRelativeAngle();
		turnDegrees(turnAmount, turnSpeedFactor);
	}

	public static void turnDegrees(double degrees, double turnSpeedFactor) {
		// Turns using the Gyro, relative to the current position
		// Use "turnCompleted" method to determine when the turn is done
		// The PID controller for this sends a rotational value to the
		// standard swerve drive method to make the bot rotate

		stopDriving();

		heading += degrees; // this is used later to help us drive straight after rotating

		SmartDashboard.putNumber("TURN DEGREES CALL", degrees);

		DriveTrain.setTurnOrientation(DriveTrain.angleToPosition(-133.6677), DriveTrain.angleToPosition(46.3322),
				DriveTrain.angleToPosition(133.6677), DriveTrain.angleToPosition(-46.3322), true);
		try {
			Thread.sleep(100);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		DriveTrain.setDriveMMVelocity((int) (Calibration.DT_MM_VELOCITY * turnSpeedFactor));

		DriveTrain.addToAllDrivePositions(convertToTicks(degreesToInches(degrees)));
	}

	// public static void continuousTurn(double degrees, double maxPower) {
	// motionStartTime = System.currentTimeMillis();
	//
	// rotDrivePID.FSetpoint(RobotGyro.getAngle() + degrees);
	// rotDrivePID.enable();
	// setRotationalPowerOutput(maxPower);
	// }
	//
	public static void continuousDrive(double inches, double maxPower) {
		setRotationalPowerOutput(maxPower);

		DriveTrain.setTurnOrientation(DriveTrain.angleToPosition(0), DriveTrain.angleToPosition(0),
				DriveTrain.angleToPosition(0), DriveTrain.angleToPosition(0), true);
		rotDrivePID.disable();
	}

	public static void tick() {
		// this is called roughly 50 times per second

		// SmartDashboard.putNumber("Cur Turn Speed", currentTurnSpeed);
		// SmartDashboard.putNumber("Decel cycles", cyclesToDecelerate);
		// SmartDashboard.putNumber("Cycles left", cyclesLeft);
		// SmartDashboard.putNumber("Int Setpoint", interimTurnSetpoint);
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
					SmartDashboard.getNumber("ROT D", Calibration.AUTO_ROT_D),
					SmartDashboard.getNumber("ROT F", Calibration.AUTO_ROT_F));

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
		return Math.abs(RobotGyro.getRelativeAngle() - heading) <= allowedError;
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

	public static void resetDriveCurrentBreaker() {
		driveCurrentBreaker.reset();
	}

	public static boolean isAgainstWall() {
		return driveCurrentBreaker.tripped();
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

//if (levitateButton) {
//    robot.levitate();
//}
