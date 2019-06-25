package frc.robot;

import java.math.BigDecimal;
import java.math.RoundingMode;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.TargetInfo.TargetType;

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
	final String autoTestEncoders = "Test Encoders";
	final String autoTeleop = "TELEOP";
	final String autoDriveOffPlatform = "Auto Platform";
	final String autoHexDrive = "Hex Drive";

	final String testProgram = "Test Program";
	final String targetTracking = "Target Tracking";

	private ShuffleboardTab visionTab = Shuffleboard.getTab("Vision");
	private NetworkTableEntry SB_vision_STR_P = visionTab.add("Vision STR P", Calibration.VISION_STR_P).getEntry();
	private NetworkTableEntry SB_vision_FWD_DIST = visionTab.add("Vision FWD DIST", 48).getEntry();

	private double currentTargetAngle = -1;
	private double rotationLockAngle = -1;

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

		DriveTrain.allowTurnEncoderReset();
		DriveTrain.resetTurnEncoders(); // sets encoders based on absolute encoder positions

		SmartDashboard.putBoolean("Show Encoders", false);
	}

	@Override
	public void teleopInit() {
		mAutoProgram.stop();
		DriveTrain.stopDriveAndTurnMotors();
		Climber.stop();
		Lift.stop();
		Manipulator.stopAll();
		DriveTrain.setAllTurnOrientation(0, false); // sets them back to calibrated zero position

		Vision.setDriverMode();
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
			Lift.resetLift();
			Manipulator.resetLinkage();
			DriveTrain.allowTurnEncoderReset();
			DriveTrain.resetTurnEncoders(); // sets encoders based on absolute encoder positions
			DriveTrain.setAllTurnOrientation(0, false);
		}

		// --------------------------------------------------
		// GAME PIECES
		// --------------------------------------------------
		if (gamepad.activateCargoIntake()) {
			System.out.println("Pressed regular cargo intake buttons");
			Manipulator.intakeCargo();
		}

		if (gamepad.activateCargoFeedIntake()) {
			Manipulator.intakeCargoFeeder();
		}

		if (gamepad.turnIntakeOff()) {
			Manipulator.stopIntake();
		}

		if (gamepad.activateHatchIntake()) {
			Manipulator.setLinkageForPlacement();
			Manipulator.intakeHatch();
		}
		if (gamepad.ejectGamePiece()) {
			Manipulator.ejectGamePiece();
			rotationLockAngle = -1;
			System.out.println("FLASH 1");
			Vision.flashLED();
		}
		if (gamepad.gamePieceOverride()) {
			Manipulator.holdGamePieceOverride();
			Vision.flashLED();
			System.out.println("FLASH 2");
		}

		if (gamepad.linkageUp()) {
			Manipulator.linkageUp();
			Lift.goToStart();
		}

		if (gamepad.linkageDown()) {
			Manipulator.linkageDown();
		}

		// --------------------------------------------------
		// LIFT
		// --------------------------------------------------

		if (Math.abs(gamepad.getManualLift()) > .1) {
			Lift.moveSetpoint(gamepad.getManualLift());
		}
		if (gamepad.goToTravelPosition()) {
			Lift.goToStart();
			Manipulator.fingerUp();
		}
		if (mAutoProgram.isRunning()) {
			if (gamepad.goToLvl1()) {
				AutoDoEverything.setLiftHeight(AutoDoEverything.LiftHeight.LVL_1);
			}
			if (gamepad.goToLvl2()) {
				AutoDoEverything.setLiftHeight(AutoDoEverything.LiftHeight.LVL_2);
			}
			if (gamepad.goToLvl3()) {
				AutoDoEverything.setLiftHeight(AutoDoEverything.LiftHeight.LVL_3);
			}
		} else {
			if (gamepad.goToLvl1()) {
				Manipulator.setLinkageForPlacement();
				if (Manipulator.isHoldingCargo())
					Lift.goCargoLvl1();
				else
					Lift.goHatchLvl1();
			}
			if (gamepad.goToLvl2()) {
				Manipulator.setLinkageForPlacement();
				if (Manipulator.isHoldingCargo())
					Lift.goCargoLvl2();
				else
					Lift.goHatchLvl2();
			}
			if (gamepad.goToLvl3()) {
				Manipulator.setLinkageForPlacement();
				if (Manipulator.isHoldingCargo())
					Lift.goCargoLvl3();
				else
					Lift.goHatchLvl3();
			}
		}

		if (gamepad.getCargoShipPlacement()) {
			Manipulator.setLinkageForPlacement();
			Lift.goCargoShipCargo();
		}

		if (gamepad.getFingerUp()) {
			Manipulator.moveFingerUp();
		}

		// if (gamepad.getHatchOverride()) {
		// 	Manipulator.intakeHatchOverride();
		// }

		// --------------------------------------------------
		// AUTO SUB ROUTINES
		// --------------------------------------------------

		// AUTO GET HATCH
		// if (gamepad.activateHatchIntakeAuto() && !mAutoProgram.isRunning()) {
		// mAutoProgram = new AutoGrabHatchFromFeeder();
		// mAutoProgram.start();
		// }

		// AUTO CLIMB
		if (gamepad.getClimb()) {
			mAutoProgram.stop();
			mAutoProgram = new AutoClimb();
			mAutoProgram.start();
		}

		if (gamepad.getUndoClimb()) {
			mAutoProgram.stop();
			Climber.climberRetractFull();
		}

		// SLIDE LEFT
		if (gamepad.shipMoveLeft() && !mAutoProgram.isRunning()) {
			mAutoProgram = new AutoSlideOver();
			mAutoProgram.start(AutoBaseClass.Direction.LEFT);
		}

		// SLIDE RIGHT
		if (gamepad.shipMoveRight() && !mAutoProgram.isRunning()) {
			mAutoProgram = new AutoSlideOver();
			mAutoProgram.start(AutoBaseClass.Direction.RIGHT);
		}

		if (gamepad.getTurnToBackTarget() && !mAutoProgram.isRunning()) {
			if (RobotGyro.getRelativeAngle() < 179 && RobotGyro.getRelativeAngle() > 0) {
				rotationLockAngle = 150; // Right back angle
			} else {
				rotationLockAngle = 210; // Left back angle
			}
			// mAutoProgram = new AutoTurn();
			// mAutoProgram.start();
		}

		// FIND HATCH TARGET
		if (gamepad.findRocketTarget() && !mAutoProgram.isRunning()) {
			mAutoProgram = new AutoDoEverything();
			((AutoDoEverything) mAutoProgram).setDrivingAllowed(true);
			((AutoDoEverything) mAutoProgram).setActionMode(AutoDoEverything.ActionMode.PLACE_HATCH);
			AutoDoEverything.setLiftHeight(AutoDoEverything.LiftHeight.LVL_1);
			mAutoProgram.start(TargetInfo.TargetType.ROCKET_TARGET);
		}
		if (gamepad.findFeedStation() && !mAutoProgram.isRunning()) {
			mAutoProgram = new AutoDoEverything();
			((AutoDoEverything) mAutoProgram).setDrivingAllowed(true);
			((AutoDoEverything) mAutoProgram).setActionMode(AutoDoEverything.ActionMode.GET_HATCH);
			AutoDoEverything.setLiftHeight(AutoDoEverything.LiftHeight.LVL_1);
			mAutoProgram.start(TargetInfo.TargetType.FEEDER_TARGET);
		}
		if (gamepad.findShipTarget() && !mAutoProgram.isRunning()) {
			mAutoProgram = new AutoDoEverything();
			((AutoDoEverything) mAutoProgram).setDrivingAllowed(true);
			((AutoDoEverything) mAutoProgram).setActionMode(AutoDoEverything.ActionMode.PLACE_HATCH);
			AutoDoEverything.setLiftHeight(AutoDoEverything.LiftHeight.LVL_1);
			mAutoProgram.start(TargetInfo.TargetType.SHIP_TARGET);
		}
		if (mAutoProgram.isRunning() && (Math.abs(gamepad.getSwerveYAxis()) > .5
				|| Math.abs(gamepad.getSwerveXAxis()) > .5 || Math.abs(gamepad.getSwerveRotAxis()) > .5)) {
			mAutoProgram.stop();
		}

		SmartDashboard.putBoolean("Auto Mode", mAutoProgram.isRunning());

		// DRIVE
		if (mAutoProgram.isRunning()) {
			mAutoProgram.tick();
		} else {
			//

			// DRIVER CONTROL MODE
			// Issue the drive command using the parameters from
			// above that have been tweaked as needed
			double driveRotAmount;
			double driveFWDAmount = gamepad.getSwerveYAxis();
			double driveStrafeAmount = -gamepad.getSwerveXAxis();
			boolean normalDrive = !gamepad.getDriveModifier();

			if (Math.abs(driveFWDAmount) <= .2 || !normalDrive) // strafe adjust if not driving forward
				driveStrafeAmount = strafeAdjust(driveStrafeAmount, normalDrive);
			// else
			// 	driveStrafeAmount = driveStrafeAmount * .50;

			// If the rotation stick is pressed all the way then disable rotation lock.
			if (Math.abs(gamepad.getSwerveRotAxis()) > 0.9) {
				rotationLockAngle = -1; // Turns off rotation lock angle and lets you turn
			}
			SmartDashboard.putNumber("Rot LOCK", rotationLockAngle);
			
			if (rotationLockAngle != -1) {
				driveRotAmount = (rotationLockAngle - RobotGyro.getRelativeAngle()) * .02;
			} else {
				driveRotAmount = rotationalAdjust(gamepad.getSwerveRotAxis());
			}

			SmartDashboard.putBoolean("AutoAssist", gamepad.getAutoAlignToTarget());

			if (gamepad.getAutoAlignToTarget()) {
				Vision.setTargetTrackingMode();

				if (currentTargetAngle == -1) {
					currentTargetAngle = TargetInfo.targetAngle(TargetType.ROCKET_TARGET);
				}

				double angleAdjust = -Vision.offsetFromTarget();
				if (angleAdjust != 0) {
					driveStrafeAmount = angleAdjust / 60; // strafeAmt is -1 to 1
					if (driveStrafeAmount < .1 && driveStrafeAmount > 0) {
						driveStrafeAmount = .1;
					}
					if (driveStrafeAmount > -.1 && driveStrafeAmount < 0) {
						driveStrafeAmount = -.1;
					}
					if (driveStrafeAmount < -1) {
						driveStrafeAmount = -1;
					}
					if (driveStrafeAmount > 1) {
						driveStrafeAmount = 1;
					}
					if (RobotGyro.getRelativeAngle() >= 90 && RobotGyro.getRelativeAngle() <= 270) {
						driveStrafeAmount = -driveStrafeAmount;
					}
				}

				if (currentTargetAngle != -1) {
					double rotAdjust = 0;// currentTargetAngle - RobotGyro.getRelativeAngle();
					if (Math.abs(rotAdjust) > 4) {
						driveRotAmount = rotAdjust * .02;
					}
				}
				//System.out.println("SA: " + driveStrafeAmount + " RA: " + driveRotAmount + " AA: " + angleAdjust);
				SmartDashboard.putNumber("AA Angle", angleAdjust);
				SmartDashboard.putNumber("AA StrafeAmount", driveStrafeAmount);
			

			} else {
				currentTargetAngle = -1;
				Vision.setDriverMode();
			}
			
			
			driveFWDAmount = forwardAdjust(driveFWDAmount, normalDrive);

			if (gamepad.getRobotCentricModifier())
				DriveTrain.humanDrive(driveFWDAmount, driveStrafeAmount, driveRotAmount);
			else
				DriveTrain.fieldCentricDrive(driveFWDAmount, driveStrafeAmount, driveRotAmount);

			if (isTippingOver()) {
				System.out.print("ANTI-TIP CODE ACTIVATED");
				Lift.goToStart(); // if we start tipping, bring the lift down
			}
		}

		// --------------------------------------------------
		// TICK
		// --------------------------------------------------

		Lift.tick();
		Climber.tick();
		Manipulator.tick();
		Vision.tick();

		showDashboardInfo();

	}

	@Override
	public void autonomousInit() {

		mAutoProgram.stop();
		Climber.stop();
		Lift.stop();
		Manipulator.stopAll();
		Vision.setDriverMode();
		DriveTrain.stopDriveAndTurnMotors();
		DriveTrain.setAllTurnOrientation(0, false);

		Lift.resetLift();
		Manipulator.resetLinkage();

		String selectedPos = positionChooser.getSelected();
		SmartDashboard.putString("Position Chooser Selected", selectedPos);
		char robotPosition = selectedPos.toCharArray()[0];

		System.out.println("Robot position: " + robotPosition);

		autoSelected = (String) autoChooser.getSelected();
		SmartDashboard.putString("Auto Selected: ", autoSelected);

		mAutoProgram = new AutoDoNothing();

		switch (autoSelected) {
		case autoTeleop:
			// don't do anything
			break;
		case autoDriveOffPlatform:
			mAutoProgram = new AutoDriveOffPlatform();
			break;
		case autoDrivePIDTune:
			SmartDashboard.putNumber("Drive To Setpoint", 0);
			SmartDashboard.putNumber("Drive Strafe Angle", 0);
			mAutoProgram = new AutoDrivePIDTune();
			break;
		case autoRotateTest:
			mAutoProgram = new AutoRotateTest();
			break;
		case autoTestEncoders:
			mAutoProgram = new AutoTestEncoders();
			break;
		case autoHexDrive:
			mAutoProgram = new AutoDriveHexagon();
			break;
		}

		DriveAuto.reset();

		if (autoSelected == autoTeleop) {
			System.out.println("AUTO TELEOP SELECTED");
		} else if (mAutoProgram != null) {
			System.out.print("auto program started successfully " + autoSelected);
			mAutoProgram.start();
		} else
			System.out.println("No auto program started in switch statement");
	}

	@Override
	public void autonomousPeriodic() {

		if (mAutoProgram != null) {
			mAutoProgram.tick();
		}

		if (autoSelected == autoTeleop) {
			teleopPeriodic();
		} else {
			DriveAuto.tick();

			DriveAuto.showEncoderValues();
			showDashboardInfo();
		}
	}

	private void showDashboardInfo() {
		// SmartDashboard.putNumber("Distance", Vision.getDistanceFromTarget());
		// visionTab.add("Has Target", Vision.targetInfoIsValid());
		// SmartDashboard.putNumber("Vision offset", Vision.offsetFromTarget());
		// SmartDashboard.putNumber("Vision Dist", Vision.getDistanceFromTarget());
		// SmartDashboard.putNumber("Vision Skew", Vision.getTargetSkew());
		// SmartDashboard.putNumber("line sensor", line.getAverageValue());

		// SmartDashboard.putNumber("Match Time",
		// DriverStation.getInstance().getMatchTime());

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

		return rotateAmt;

		// double adjustedAmt = 0;

		// if (Math.abs(rotateAmt) < .1) {
		// 	adjustedAmt = 0;
		// } else {
		// 	if (Math.abs(rotateAmt) < .6) {
		// 		adjustedAmt = .10 * Math.signum(rotateAmt);
		// 	} else {
		// 		if (Math.abs(rotateAmt) < .8) {
		// 			adjustedAmt = .20 * Math.signum(rotateAmt);
		// 		} else {
		// 			if (Math.abs(rotateAmt) < .95) {
		// 				adjustedAmt = .45 * Math.signum(rotateAmt);
		// 			} else {
		// 				adjustedAmt = rotateAmt * .65;
		// 			}
		// 		}
		// 	}
		// }
		// return adjustedAmt;
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
		autoChooser.addOption(autoTestEncoders, autoTestEncoders);
		autoChooser.addOption(autoDriveOffPlatform, autoDriveOffPlatform);
		autoChooser.setDefaultOption(autoTeleop, autoTeleop);
		autoChooser.addOption(autoHexDrive, autoHexDrive);

		// Put options to smart dashboard
		SmartDashboard.putData("Auto choices", autoChooser);
	}

	private boolean isTippingOver() {
		return Math.abs(RobotGyro.getGyro().getPitch()) > 20 || Math.abs(RobotGyro.getGyro().getRoll()) > 20;
	}

	private double powerOf2PreserveSign(double v) {
		return (v > 0) ? Math.pow(v, 2) : -Math.pow(v, 2);
	}

	private static Double round2(Double val) {
		// added this back in on 1/15/18
		return new BigDecimal(val.toString()).setScale(2, RoundingMode.HALF_UP).doubleValue();
	}

	private static Double round0(Double val) {
		// added this back in on 1/15/18
		return new BigDecimal(val.toString()).setScale(0, RoundingMode.HALF_UP).doubleValue();
	}
}

// if (!win) {
// this.win = true;
// }

// while (win) {
// robot.celebrate
// }
