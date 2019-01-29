
package frc.robot;

import java.math.BigDecimal;
import java.math.RoundingMode;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

	// Setup stuff
	KeyMap gamepad;
	Compressor compressor;
	SendableChooser<String> autoChooser;
	SendableChooser<String> positionChooser;
	AnalogInput line;
	final String cargoTracking = "Cargo Tracking";

	private boolean inAutoMode = false;

	private PIDController pidDistance;
	private PIDController pidStrafe;
	private PIDController pidRotation;

	private FWDOutput fwdOutput;
	private STROutput strOutput;
	private ROTOutput rotOutput;

	private STRVision strVision;
	private FWDVision fwdVision;
	private ROTVision rotVision;

	// /* Auto Stuff */
	String autoSelected;
	AutoBaseClass mAutoProgram;
	// Auto options
	final String testProgram = "Test Program";
	final String targetTracking = "Target Tracking";

	// End setup stuff

	@Override
	public void robotInit() {
		// Position Chooser
		positionChooser = new SendableChooser<String>();
		positionChooser.addObject("Left", "L");
		positionChooser.addDefault("Center", "C");
		positionChooser.addObject("Right", "R");
		SmartDashboard.putData("Position", positionChooser);

		/* Auto Chooser */
		autoChooser = new SendableChooser<>();
		autoChooser.addOption(targetTracking, targetTracking);
		// Put options to smart dashboard
		SmartDashboard.putData("Auto choices", autoChooser);

		gamepad = new KeyMap();

		RobotGyro.getInstance();
		DriveTrain.getInstance();
		DriveAuto.getInstance();

		fwdOutput = new FWDOutput();
		strOutput = new STROutput();
		rotOutput = new ROTOutput();

		fwdVision = new FWDVision();
		strVision = new STRVision();
		rotVision = new ROTVision();

		pidDistance = new PIDController(Calibration.VISION_FWD_P, Calibration.VISION_FWD_I, Calibration.VISION_FWD_D,
				fwdVision, fwdOutput);
		pidStrafe = new PIDController(Calibration.VISION_STR_P, Calibration.VISION_STR_I, Calibration.VISION_STR_D,
				strVision, strOutput);
		pidRotation = new PIDController(Calibration.VISION_ROT_P, Calibration.VISION_ROT_I, Calibration.VISION_ROT_D,
				rotVision, rotOutput);

		Calibration.loadSwerveCalibration();

		line = new AnalogInput(0);

		compressor = new Compressor(0);
		compressor.setClosedLoopControl(true);

		DriveTrain.setDrivePIDValues(Calibration.AUTO_DRIVE_P, Calibration.AUTO_DRIVE_I, Calibration.AUTO_DRIVE_D);

		RobotGyro.reset(); // this is also done in auto init in case it wasn't
							// settled here yet

		SmartDashboard.putBoolean("Show Turn Encoders", true);

		SmartDashboard.putNumber("Vision FWD P", Calibration.VISION_FWD_P);
		SmartDashboard.putNumber("Vision FWD I", Calibration.VISION_FWD_I);
		SmartDashboard.putNumber("Vision FWD D", Calibration.VISION_FWD_D);

		SmartDashboard.putNumber("Vision STR P", Calibration.VISION_STR_P);
		SmartDashboard.putNumber("Vision STR I", Calibration.VISION_STR_I);
		SmartDashboard.putNumber("Vision STR D", Calibration.VISION_STR_D);

		SmartDashboard.putNumber("Vision ROT P", Calibration.VISION_ROT_P);
		SmartDashboard.putNumber("Vision ROT I", Calibration.VISION_ROT_I);
		SmartDashboard.putNumber("Vision ROT D", Calibration.VISION_ROT_D);

		// SmartDashboard.putNumber("Auto P:", Calibration.AUTO_DRIVE_P);
		// SmartDashboard.putNumber("Auto I:", Calibration.AUTO_DRIVE_I);
		// SmartDashboard.putNumber("Auto D:", Calibration.AUTO_DRIVE_D);

	}

	/*
	 * 
	 * TELEOP PERIODIC
	 * 
	 */

	@Override
	public void teleopPeriodic() {

		adjustPIDs();

		SmartDashboard.putNumber("Version", 2.8);

		double driveYAxisAmount = gamepad.getSwerveYAxis();
		double driveXAxisAmount = -gamepad.getSwerveXAxis();
		double driveRotAxisAmount = powerOf3PreserveSign(gamepad.getSwerveRotAxis());

		// put some rotational power restrictions in place to make it
		// more controlled
		if (Math.abs(driveRotAxisAmount) > .70) {
			if (driveRotAxisAmount < 0)
				driveRotAxisAmount = -.70;
			else
				driveRotAxisAmount = .70;
		}

		// A
		if (gamepad.getButtonA(0)) {

			inAutoMode = true;

			Vision.setTargetTrackingMode();

			pidDistance.setSetpoint(72);
			pidDistance.enable();

			pidStrafe.setSetpoint(0);
			pidStrafe.enable();

			pidRotation.setSetpoint(0);
			pidRotation.enable();
		}
		// B
		if (gamepad.getButtonB(0)) {

			inAutoMode = false;

			Vision.setDriverMode();

			pidDistance.disable();
			pidStrafe.disable();
			pidRotation.disable();
		}

		if (inAutoMode) {

			double fwd = fwdOutput.getValue();
			double str = strOutput.getValue();
			double rot = rotOutput.getValue();

			if (Vision.targetInfoIsValid()) { // we're in auto mode and have a target in sight
				pidDistance.enable();
				pidStrafe.enable();
				pidRotation.enable();
			} else { // we're in auto, but have no target, so disable movement.
				pidDistance.disable();
				pidStrafe.disable();
				pidRotation.disable();
				fwd = 0;
				str = 0;
				rot = 0;
			}

			DriveTrain.swerveDrive(fwd, str, rot);

			SmartDashboard.putNumber("fwd", fwd);
			SmartDashboard.putNumber("str", str);
			SmartDashboard.putNumber("rot", rot);

		} else {
			// DRIVER CONTROL MODE
			// Issue the drive command using the parameters from
			// above that have been tweaked as needed

			DriveTrain.fieldCentricDrive(driveYAxisAmount, driveXAxisAmount, driveRotAxisAmount);
		}

		showDashboardInfo();

		Vision.tick();

	}

	private void showDashboardInfo() {
		SmartDashboard.putNumber("Distance", Vision.getDistanceFromTarget());

		SmartDashboard.putNumber("Vision offset", Vision.offsetFromTarget());
		SmartDashboard.putNumber("Vision Area", Vision.targetArea());

		SmartDashboard.putNumber("line sensor", line.getAverageValue());

		SmartDashboard.putNumber("Match Time", DriverStation.getInstance().getMatchTime());

		SmartDashboard.putNumber("Gyro Heading", round0(RobotGyro.getAngle()));

		if (SmartDashboard.getBoolean("Show Turn Encoders", false)) {
			DriveTrain.showTurnEncodersOnDash();
			DriveTrain.showDriveEncodersOnDash();
		}

	}

	private void adjustPIDs() {

		pidDistance.setPID(SmartDashboard.getNumber("Vision FWD P", Calibration.VISION_FWD_P),
				SmartDashboard.getNumber("Vision FWD I", Calibration.VISION_FWD_I),
				SmartDashboard.getNumber("Vision FWD D", Calibration.VISION_FWD_D));

		pidStrafe.setPID(SmartDashboard.getNumber("Vision STR P", Calibration.VISION_STR_P),
				SmartDashboard.getNumber("Vision STR I", Calibration.VISION_STR_I),
				SmartDashboard.getNumber("Vision STR D", Calibration.VISION_STR_D));

		pidRotation.setPID(SmartDashboard.getNumber("Vision ROT P", Calibration.VISION_ROT_P),
				SmartDashboard.getNumber("Vision ROT I", Calibration.VISION_ROT_I),
				SmartDashboard.getNumber("Vision ROT D", Calibration.VISION_ROT_D));

		// in case we're tweaking the drive turn PID via Dashboard, update the values
		DriveTrain.setTurnPIDValues(SmartDashboard.getNumber("TURN P", Calibration.TURN_P),
				SmartDashboard.getNumber("TURN I", Calibration.TURN_I),
				SmartDashboard.getNumber("TURN D", Calibration.TURN_D));
	}

	@Override
	public void autonomousInit() {

		// String selectedPos = positionChooser.getSelected();
		// SmartDashboard.putString("Position Chooser Selected", selectedPos);
		// char robotPosition = selectedPos.toCharArray()[0];
		// System.out.println("Robot position: " + robotPosition);

		// mAutoProgram = null;

		// switch (autoSelected) {
		// case targetTracking:
		// mAutoProgram = new TargetTracking('C');
		// break;
		// }

		// DriveTrain.setAllTurnOrientiation(0);
		// DriveAuto.reset();

		// if (mAutoProgram != null) {
		// mAutoProgram.start();
		// } else {
		// System.out.println("No auto program started in switch statement");
		// }
	}

	@Override
	public void autonomousPeriodic() {

		// DriveAuto.tick();
		// mAutoProgram = new TargetTracking('C');
		// mAutoProgram.start();
	}

	@Override
	public void teleopInit() {
		DriveAuto.stop();
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

	}

	public void disabledPeriodic() {
		DriveTrain.resetTurnEncoders(); // happens only once because a flag
										// prevents multiple calls
		DriveTrain.disablePID();

		SmartDashboard.putNumber("Gyro PID Get", round0(RobotGyro.getInstance().pidGet()));

		if (SmartDashboard.getBoolean("Show Turn Encoders", false)) {
			DriveTrain.showTurnEncodersOnDash();
		}

	}

	// Is this used?
	private double powerOf2PreserveSign(double v) {
		return (v > 0) ? Math.pow(v, 2) : -Math.pow(v, 2);
	}

	// Is this used?
	private double powerOf3PreserveSign(double v) {
		return Math.pow(v, 3);
	}

	// Is this used?
	private static Double round2(Double val) {
		// added this back in on 1/15/18
		return new BigDecimal(val.toString()).setScale(2, RoundingMode.HALF_UP).doubleValue();
	}

	// Is this used?
	private static Double round0(Double val) {
		// added this back in on 1/15/18
		return new BigDecimal(val.toString()).setScale(0, RoundingMode.HALF_UP).doubleValue();
	}

	/// Auto Crap
	// This is sorta used.
	public void driveInches(double distance, double angle, double maxPower) {
		DriveAuto.driveInches(distance, angle, maxPower);
	}

	public void turnDegrees(double degrees, double maxPower) {
		DriveAuto.turnDegrees(degrees, maxPower);
	}
}
