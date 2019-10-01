package frc.robot;

import java.math.BigDecimal;
import java.math.RoundingMode;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
	KeyMap gamepad;

	@Override
	public void robotInit() {

		gamepad = new KeyMap();

		RobotGyro.getInstance();
		DriveTrain.getInstance();
	
		Calibration.loadSwerveCalibration();

		RobotGyro.reset();

		DriveTrain.allowTurnEncoderReset();
		DriveTrain.resetTurnEncoders(); // sets encoders based on absolute encoder positions

		SmartDashboard.putBoolean("Show Encoders", false);
	}

	@Override
	public void teleopInit() {
		DriveTrain.stopDriveAndTurnMotors();
		DriveTrain.setAllTurnOrientation(0, false); // sets them back to calibrated zero position
	}

	/*
	 * 
	 * TELEOP PERIODIC
	 * 
	 */

	@Override
	public void teleopPeriodic() {

		// --------------------------------------------------
		// RESST - allow manual reset of systems by pressing Start
		// --------------------------------------------------
		if (gamepad.getZeroGyro()) {
			RobotGyro.reset();
			DriveTrain.allowTurnEncoderReset();
			DriveTrain.resetTurnEncoders(); // sets encoders based on absolute encoder positions
			DriveTrain.setAllTurnOrientation(0, false);
		}


		// DRIVER CONTROL MODE
		// Issue the drive command using the parameters from
		// above that have been tweaked as needed
		double driveRotAmount;
		double driveFWDAmount = gamepad.getSwerveYAxis();
		double driveStrafeAmount = -gamepad.getSwerveXAxis();
		boolean normalDrive = !gamepad.getDriveModifier();

		if (Math.abs(driveFWDAmount) <= .2 || !normalDrive) // strafe adjust if not driving forward
			driveStrafeAmount = strafeAdjust(driveStrafeAmount, normalDrive);

		driveRotAmount = rotationalAdjust(gamepad.getSwerveRotAxis());

		driveFWDAmount = forwardAdjust(driveFWDAmount, normalDrive);

		if (gamepad.getRobotCentricModifier())
			DriveTrain.humanDrive(driveFWDAmount, driveStrafeAmount, driveRotAmount);
		else
			DriveTrain.fieldCentricDrive(driveFWDAmount, driveStrafeAmount, driveRotAmount);

		showDashboardInfo();
	}
	

	@Override
	public void autonomousInit() {
	}

	@Override
	public void autonomousPeriodic() {
	}

	private void showDashboardInfo() {
		SmartDashboard.putNumber("Gyro Relative", round2(RobotGyro.getRelativeAngle()));
		SmartDashboard.putNumber("Gyro Raw", round2(RobotGyro.getAngle()));

		if (SmartDashboard.getBoolean("Show Encoders", false)) {
			DriveTrain.showTurnEncodersOnDash();
			DriveTrain.showDriveEncodersOnDash();
		}

	}

	private double rotationalAdjust(double rotateAmt) {
		// put some rotational power restrictions in place to make it
		// more controlled movement

		// return rotateAmt;

		double adjustedAmt = 0;

		if (Math.abs(rotateAmt) < .1) {
			adjustedAmt = 0;
		} else {
			if (Math.abs(rotateAmt) < .6) {
				adjustedAmt = .10 * Math.signum(rotateAmt);
			} else {
				if (Math.abs(rotateAmt) < .8) {
					adjustedAmt = .30 * Math.signum(rotateAmt);
				} else {
					if (Math.abs(rotateAmt) < .95) {
						adjustedAmt = .45 * Math.signum(rotateAmt);
					} else {
						adjustedAmt = rotateAmt * .85;
					}
				}
			}
		}
		return adjustedAmt;
	}

	private double forwardAdjust(double fwd, boolean normalDrive) {
		if (normalDrive) {
			return fwd;
		} else {
			return fwd * .45;
		}
	}

	private double strafeAdjust(double strafeAmt, boolean normalDrive) {
		// put some power restrictions in place to make it
		// more controlled
		return strafeAmt;

		// double adjustedAmt = 0;

		// if (Math.abs(strafeAmt) < .1) {
		// 	adjustedAmt = 0;
		// } else {
		// 	if (normalDrive) { // do normal adjustments
		// 		if (Math.abs(strafeAmt) < .7) {
		// 			adjustedAmt = .3 * strafeAmt; // .2 * Math.signum(strafeAmt);
		// 		} else {
		// 			if (Math.abs(strafeAmt) < .98) {
		// 				adjustedAmt = .50 * strafeAmt; // .4 * Math.signum(strafeAmt);
		// 			} else {
		// 				adjustedAmt = strafeAmt;
		// 			}
		// 		}
		// 	} else { // lift is up, so do more drastic adjustments
		// 		adjustedAmt = strafeAmt * .35;
		// 	}
		// }
		// return adjustedAmt;
	}

	@Override
	public void testInit() {
	}

	@Override
	public void testPeriodic() {
	}

	public void disabledInit() {
		DriveTrain.allowTurnEncoderReset(); // allows the turn encoders to be
											// reset once during disabled
											// periodic
		DriveTrain.resetDriveEncoders();

		DriveTrain.disablePID();
	}

	public void disabledPeriodic() {
		DriveTrain.resetTurnEncoders(); // happens only once because a flag
										// prevents multiple calls
		showDashboardInfo();
	}

	private static Double round2(Double val) {
		// added this back in on 1/15/18
		return new BigDecimal(val.toString()).setScale(2, RoundingMode.HALF_UP).doubleValue();
	}

}
