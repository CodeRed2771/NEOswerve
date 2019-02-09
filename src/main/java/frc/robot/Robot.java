
package frc.robot;

import java.math.BigDecimal;
import java.math.RoundingMode;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.libs.PIDSourceFilter.PIDGetFilter;

public class Robot extends TimedRobot {

	WPI_TalonSRX leftInake = new WPI_TalonSRX(10);
	WPI_TalonSRX rightIntake = new WPI_TalonSRX(11);
	// private WPI_TalonSRX leftIntake

	// Setup stuff
	KeyMap gamepad;
	Compressor compressor;
	SendableChooser<String> autoChooser;
	SendableChooser<String> positionChooser;
	AnalogInput line;
	final String cargoTracking = "Cargo Tracking";

	private boolean inAutoMode = false;
	private boolean isAutoDriving = false;
	private boolean isAligned = false;
	private boolean isTurning = false;

	private boolean pidRotationEnabled = false;
	private boolean pidStrafeEnabled = false;

	// /* Auto Stuff */
	String autoSelected;
	AutoBaseClass mAutoProgram;
	// Auto options
	final String autoRotateTest = "Rotate Test";
	final String autoCalibrateDrive = "Auto Calibrate Drive";
	final String autoDrivePIDTune = "Drive PID Tune";

	final String testProgram = "Test Program";
	final String targetTracking = "Target Tracking";

	private ShuffleboardTab visionTab = Shuffleboard.getTab("Vision");
	private NetworkTableEntry SB_vision_STR_P = visionTab.add("Vision STR P", Calibration.VISION_STR_P).getEntry();
	private NetworkTableEntry SB_vision_FWD_DIST = visionTab.add("Vision FWD DIST", 48).getEntry();

	// End setup stuff

	@Override
	public void robotInit() {
		leftInake = new WPI_TalonSRX(10);
		rightIntake = new WPI_TalonSRX(11);

		// Position Chooser
		positionChooser = new SendableChooser<String>();
		positionChooser.addObject("Left", "L");
		positionChooser.addDefault("Center", "C");
		positionChooser.addObject("Right", "R");
		SmartDashboard.putData("Position", positionChooser);

		/* Auto Chooser */
		autoChooser = new SendableChooser<>();
		autoChooser.addOption(targetTracking, targetTracking);
		autoChooser.addObject(autoRotateTest, autoRotateTest);
		autoChooser.addObject(autoCalibrateDrive, autoCalibrateDrive);

		// Put options to smart dashboard
		SmartDashboard.putData("Auto choices", autoChooser);

		gamepad = new KeyMap();

		RobotGyro.getInstance();
		DriveTrain.getInstance();
		DriveAuto.getInstance();
		TargetInfo.getInstance();

		Calibration.loadSwerveCalibration();

		line = new AnalogInput(0);

		// compressor = new Compressor(0);
		// compressor.setClosedLoopControl(true);

		DriveTrain.setDrivePIDValues(Calibration.AUTO_DRIVE_P, Calibration.AUTO_DRIVE_I, Calibration.AUTO_DRIVE_D);

		RobotGyro.reset(); // this is also done in auto init in case it wasn't
							// settled here yet

		SmartDashboard.putBoolean("Show Turn Encoders", false);

		// SmartDashboard.putNumber("Vision FWD P", Calibration.VISION_FWD_P);
		// SmartDashboard.putNumber("Vision FWD I", Calibration.VISION_FWD_I);
		// SmartDashboard.putNumber("Vision FWD D", Calibration.VISION_FWD_D);

		// // SmartDashboard.putNumber("Vision STR P", Calibration.VISION_STR_P);
		// SmartDashboard.putNumber("Vision STR I", Calibration.VISION_STR_I);
		// SmartDashboard.putNumber("Vision STR D", Calibration.VISION_STR_D);

		// SmartDashboard.putNumber("Vision ROT P", Calibration.VISION_ROT_P);
		// SmartDashboard.putNumber("Vision ROT I", Calibration.VISION_ROT_I);
		// SmartDashboard.putNumber("Vision ROT D", Calibration.VISION_ROT_D);

		// SmartDashboard.putBoolean("FWD seeking enabled", true);
		// SmartDashboard.putBoolean("STR seeking enabled", true);
		// SmartDashboard.putBoolean("ROT seeking enabled", true);

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
		SmartDashboard.putNumber("TURN A RAW", DriveTrain.round(DriveTrain.moduleA.getTurnAbsolutePosition(), 3));

		// allow manual gyro reset if you press Start button
		if (gamepad.getStartButton(0)) {
			RobotGyro.reset();
		}

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
		// X
		if (gamepad.getButtonX(0)) {
			leftInake.set(0.5);
			rightIntake.set(0.5);
		}
		// Y
		if (gamepad.getButtonY(0)) {
			Vision.setTargetTrackingMode();	
		}

		// A
		if (gamepad.getButtonA(0)) {
			inAutoMode = true;	
			Vision.setTargetTrackingMode();
			DriveAuto.setPIDstate(true);
		}
		// B
		if (gamepad.getButtonB(0)) {
			inAutoMode = false;
			isAutoDriving = false;
			isAligned = false;
			isTurning = false;
			DriveAuto.stop();

			Vision.setDriverMode();
		}

		SmartDashboard.putBoolean("Is Auto Driving", isAutoDriving);

		if (inAutoMode) {
			double dist = Vision.getDistanceFromTarget();
			if(!isAligned && dist > 0 && !isTurning){
				DriveAuto.turnDegrees(Vision.offsetFromTarget(), .2);
				isTurning = true;
			}
			if(Math.abs(Vision.offsetFromTarget()) < 2 && !isAutoDriving){
				isAligned = true;
				isTurning = false;
				DriveAuto.stop();
			}
			if (!isAutoDriving && isAligned) { // have valid target
				double distToStayBack = 36;
				// TO DO - use Vision rotation to center on target
				double offSet = Vision.offsetFromTarget();
				double angleDiff = TargetInfo.targetAngle() - RobotGyro.getAngle();
				double opposite = Math.sin(angleDiff) * dist; 
				double adjacent = Math.cos(angleDiff) * dist;

				double newDist = Math.sqrt(Math.pow(opposite,2) + Math.pow((adjacent - distToStayBack),2));
				double newAngle = Math.atan(opposite/(adjacent - distToStayBack));

				isAutoDriving = true;
				
				SmartDashboard.putNumber("orig dist", dist);			
				SmartDashboard.putNumber("Angle Diff", angleDiff);
				SmartDashboard.putNumber("opposite", opposite);
				SmartDashboard.putNumber("adjacent", adjacent);
				SmartDashboard.putNumber("Drive dist", newDist);
				SmartDashboard.putNumber("Drive angle", newAngle);
	
				//DriveAuto.reset();
				DriveAuto.driveInches(newDist/2, 0, .8); // 20 is temp hardcoded

			}
		} else {
			// DRIVER CONTROL MODE
			// Issue the drive command using the parameters from
			// above that have been tweaked as needed

			DriveTrain.fieldCentricDrive(driveYAxisAmount, driveXAxisAmount, driveRotAxisAmount);
		}

		showDashboardInfo();

		try {
			SmartDashboard.putNumber("Drive setpoint", DriveTrain.moduleA.drive.getClosedLoopTarget(0));
			SmartDashboard.putNumber("Drive encoder", DriveTrain.moduleA.drive.getSelectedSensorPosition());
			SmartDashboard.putNumber("Drive PID error", DriveTrain.moduleA.drive.getClosedLoopError());
			SmartDashboard.putBoolean("Is turning", isTurning);
		} catch (Exception ex) {
			System.out.println("Error sending to shuffleboard");
			System.out.println(ex.getMessage());
			System.out.println(ex.getStackTrace());
		}
		

		Vision.tick();
		DriveAuto.tick();

	}

	private void showDashboardInfo() {
		SmartDashboard.putNumber("Distance", Vision.getDistanceFromTarget());

		// visionTab.add("Has Target", Vision.targetInfoIsValid());

		// SmartDashboard.putNumber("Vision offset", Vision.offsetFromTarget());
		// SmartDashboard.putNumber("Vision Dist", Vision.getDistanceFromTarget());
		SmartDashboard.putNumber("Vision Skew", Vision.getTargetSkew());
		SmartDashboard.putNumber("line sensor", line.getAverageValue());

		SmartDashboard.putNumber("Match Time", DriverStation.getInstance().getMatchTime());

		SmartDashboard.putNumber("Gyro Heading", round0(RobotGyro.getAngle()));

		if (SmartDashboard.getBoolean("Show Turn Encoders", false)) {
			DriveTrain.showTurnEncodersOnDash();
			DriveTrain.showDriveEncodersOnDash();
		}

	}

	private void adjustPIDs() {

		// pidDistance.setPID(SmartDashboard.getNumber("Vision FWD P",
		// Calibration.VISION_FWD_P),
		// SmartDashboard.getNumber("Vision FWD I", Calibration.VISION_FWD_I),
		// SmartDashboard.getNumber("Vision FWD D", Calibration.VISION_FWD_D));

		// pidStrafe.setPID(SB_vision_STR_P.getDouble(Calibration.VISION_STR_P),
		// SmartDashboard.getNumber("Vision STR I", Calibration.VISION_STR_I),
		// SmartDashboard.getNumber("Vision STR D", Calibration.VISION_STR_D));

		// pidRotation.setPID(SmartDashboard.getNumber("Vision ROT P",
		// Calibration.VISION_ROT_P),
		// SmartDashboard.getNumber("Vision ROT I", Calibration.VISION_ROT_I),
		// SmartDashboard.getNumber("Vision ROT D", Calibration.VISION_ROT_D));

		// in case we're tweaking the drive turn PID via Dashboard, update the values
		DriveTrain.setTurnPIDValues(SmartDashboard.getNumber("TURN P", Calibration.TURN_P),
				SmartDashboard.getNumber("TURN I", Calibration.TURN_I),
				SmartDashboard.getNumber("TURN D", Calibration.TURN_D));
	}

	@Override
	public void autonomousInit() {

		String selectedPos = positionChooser.getSelected();
		SmartDashboard.putString("Position Chooser Selected", selectedPos);
		char robotPosition = selectedPos.toCharArray()[0];

		System.out.println("Robot position: " + robotPosition);

		autoSelected = (String) autoChooser.getSelected();
		SmartDashboard.putString("Auto Selected: ", autoSelected);

		mAutoProgram = null;

		switch (autoSelected) {
			case autoDrivePIDTune:
				SmartDashboard.putNumber("Drive To Setpoint", 0);
				mAutoProgram = new AutoDrivePIDTune(robotPosition);
				break;
			case autoRotateTest:
				mAutoProgram = new AutoRotateTest(robotPosition);
				break;
		}

		DriveTrain.setAllTurnOrientiation(0);
		DriveAuto.reset();

		if (mAutoProgram != null) {
			mAutoProgram.start();
		} else
			System.out.println("No auto program started in switch statement");
	}

	@Override
	public void autonomousPeriodic() {

		if (mAutoProgram != null) {
			mAutoProgram.tick();
		}

		DriveAuto.tick();

		DriveAuto.showEncoderValues();

		SmartDashboard.putNumber("Gyro PID Get", round0(RobotGyro.getInstance().pidGet()));

	}

	@Override
	public void teleopInit() {

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
