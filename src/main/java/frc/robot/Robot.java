
package frc.robot;

import java.math.BigDecimal;
import java.math.RoundingMode;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
	KeyMap gamepad;

	SendableChooser<String> autoChooser;
	SendableChooser<String> positionChooser;

	// /* Auto Stuff */
	String autoSelected;
	AutoBaseClass mAutoProgram;
	// Auto options
	final String cargoTracking = "Cargo Tracking";
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

		gamepad = new KeyMap();

		RobotGyro.getInstance();
		DriveTrain.getInstance();
		DriveAuto.getInstance();
		TargetInfo.getInstance();
		Lift.getInstance();
		Climber.getInstance();
		Manipulator.getInstance();

		mAutoProgram = new AutoDoNothing();

		Calibration.loadSwerveCalibration();

		setupAutoChoices();

		RobotGyro.reset();

		SmartDashboard.putBoolean("Show Encoders", false);
	}

	/*
	 * 
	 * TELEOP PERIODIC
	 * 
	 */

	@Override
	public void teleopPeriodic() {

		// --------------------------------------------------
		// GYRO - allow manual gyro reset by pressing Start 
		// --------------------------------------------------
		if (gamepad.getStartButton(0)) {
			RobotGyro.reset();
		}

		// --------------------------------------------------
		//    CLIMB
		// --------------------------------------------------
		// Manual controls
		if (gamepad.getSingleClimbRevolution()) {
			Climber.moveSetpoint(1);
		}
		if (gamepad.getSingleClimbReverseRevolution()) {
			Climber.moveSetpoint(-1);
		}

		// --------------------------------------------------
		//   CARGO
		// --------------------------------------------------
		if (gamepad.activateCargoIntake()) {
			Manipulator.intakeCargo();
		}
		if (gamepad.ejectCargo()) {
			Manipulator.ejectGamePiece();
		}

		// --------------------------------------------------
		//   LIFT
		// --------------------------------------------------

		if (gamepad.getManualLiftUp()) {
			Lift.moveSetpoint(-1);
		}
		if (gamepad.getManualLiftDown()) {
			Lift.moveSetpoint(1);
		}
		if (gamepad.getBringLiftToStart()) {
			Lift.goToStart();
		}
		if (gamepad.getHatchRocketLvl1()) {
			Lift.goHatchLvl1();
		}
		if (gamepad.getHatchRocketLvl2()) {
			Lift.goHatchLvl2();
		}
		if (gamepad.getHatchRocketLvl3()) {
			Lift.goHatchLvl3();
		}
		if (gamepad.getCargoRocketLvl1()) {
			Lift.goCargoLvl1();
		}
		if (gamepad.getCargoRocketLvl2()) {
			Lift.goCargoLvl2();
		}
		if (gamepad.getCargoRocketLvl3()) {
			Lift.goCargoLvl3();
		}
		if (gamepad.getcargoShipPlacement()) {
			Lift.goCargoShipCargo();
		}

		// --------------------------------------------------
		//   LINKAGE
		// --------------------------------------------------
		// Manipulator.linkageMove(gamepad.getManualLinkage());

		// if (gamepad.getLinkageUp()) {
		// Manipulator.linkageUp();
		// }

		// if (gamepad.getLinkageDown()) {
		// Manipulator.linkageDown();
		// }

		// --------------------------------------------------
		//   AUTO SUB ROUTINES
		// --------------------------------------------------
		
		// DRIVE OFF PLATFORM
		if (gamepad.getButtonX(0)) {
			mAutoProgram = new AutoDriveOffPlatform();
			mAutoProgram.start(positionChooser.getSelected().toCharArray()[0]);
		}

		// SLIDE LEFT
		if (gamepad.getDpadLeft(0) && !mAutoProgram.isRunning()) {
			mAutoProgram = new AutoSlideOver();
			mAutoProgram.start(AutoBaseClass.Direction.LEFT);
		}

		// SLIDE RIGHT
		if (gamepad.getDpadRight(0) && !mAutoProgram.isRunning()) {
			mAutoProgram = new AutoSlideOver();
			mAutoProgram.start(AutoBaseClass.Direction.RIGHT);
		}

		// FIND HATCH TARGET
		// if (gamepad.getButtonA(0) && !mAutoProgram.isRunning()) {
		// mAutoProgram = new AutoFindHatch();
		// mAutoProgram.start();
		// }

		// STOP ANY AUTO ROUTINE
		if (gamepad.getButtonB(0)) {
			mAutoProgram.stop();
		}

		SmartDashboard.putBoolean("Auto Running", mAutoProgram.isRunning());

		// DRIVE 
		if (mAutoProgram.isRunning()) {
			mAutoProgram.tick();
		} else {
			// 
			// DRIVER CONTROL MODE
			// Issue the drive command using the parameters from
			// above that have been tweaked as needed
			double driveYAxisAmount = gamepad.getSwerveYAxis();
			double driveXAxisAmount = -gamepad.getSwerveXAxis();
			double driveRotAxisAmount = rotationalAdjust(gamepad.getSwerveRotAxis());

			DriveTrain.fieldCentricDrive(driveYAxisAmount, driveXAxisAmount, driveRotAxisAmount);
		}

		// --------------------------------------------------
		//   TESTING
		// --------------------------------------------------
		
		// Y
		if (gamepad.getButtonY(0)) {
			Vision.setTargetTrackingMode();
		}

		// --------------------------------------------------
		//   TICK 
		// --------------------------------------------------

		Lift.tick();
		Climber.tick();
		Manipulator.tick();
		// Vision.tick();

		showDashboardInfo();

	}

	private void showDashboardInfo() {
		// SmartDashboard.putNumber("Distance", Vision.getDistanceFromTarget());
		// visionTab.add("Has Target", Vision.targetInfoIsValid());
		// SmartDashboard.putNumber("Vision offset", Vision.offsetFromTarget());
		// SmartDashboard.putNumber("Vision Dist", Vision.getDistanceFromTarget());
		// SmartDashboard.putNumber("Vision Skew", Vision.getTargetSkew());
		// SmartDashboard.putNumber("line sensor", line.getAverageValue());

		SmartDashboard.putNumber("Match Time", DriverStation.getInstance().getMatchTime());

		SmartDashboard.putNumber("Gyro", round2(RobotGyro.getAngle()));

		if (SmartDashboard.getBoolean("Show Encoders", false)) {
			DriveTrain.showTurnEncodersOnDash();
			DriveTrain.showDriveEncodersOnDash();
		}

	}

	private double rotationalAdjust(double rotateAmt) {
		// put some rotational power restrictions in place to make it
		// more controlled
		double adjustedAmt = 0;

		if (Math.abs(rotateAmt) < .2) {
			adjustedAmt = 0;
		} else {
			if (Math.abs(rotateAmt) < .6) {
				adjustedAmt = .3 * Math.signum(rotateAmt);
			} else {
				if (Math.abs(rotateAmt) < .95) {
					adjustedAmt = .6 * Math.signum(rotateAmt);
				} else {
					adjustedAmt = rotateAmt;
				}
			}
		}
		return adjustedAmt;
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
			mAutoProgram = new AutoDrivePIDTune();
			break;
		case autoRotateTest:
			mAutoProgram = new AutoRotateTest();
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
		showDashboardInfo();

	}

	@Override
	public void teleopInit() {
		mAutoProgram.stop();
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

		showDashboardInfo();
	}

	private void setupAutoChoices() {
		// Position Chooser
		positionChooser = new SendableChooser<String>();
		positionChooser.addOption("Left", "L");
		positionChooser.setDefaultOption("Center", "C");
		positionChooser.addOption("Right", "R");
		SmartDashboard.putData("Position", positionChooser);

		/* Auto Chooser */
		autoChooser = new SendableChooser<>();
		autoChooser.addOption(targetTracking, targetTracking);
		autoChooser.addOption(autoRotateTest, autoRotateTest);
		autoChooser.addOption(autoCalibrateDrive, autoCalibrateDrive);
		autoChooser.addOption(autoDrivePIDTune, autoDrivePIDTune);

		// Put options to smart dashboard
		SmartDashboard.putData("Auto choices", autoChooser);
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

}
