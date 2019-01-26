
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
import edu.wpi.first.networktables.*;

public class Robot extends TimedRobot {

	// Settup stuff
	KeyMap gamepad;
	Compressor compressor;
	SendableChooser<String> autoChooser;
	SendableChooser<String> positionChooser;
	AnalogInput line;
	final String cargoTracking = "Cargo Tracking";

	private boolean inAutoMode = false;

	private PIDController pidDistance;
	private PIDController pidStrafe;

	private Vision pidVision;
	private double kVisionP = .2;
	private double kVisionD = 0.03;

	private FWDOutput fwdOutput;
	private STROutput strOutput;

	private STRVision strVision;

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
		pidVision = Vision.getInstance();

		fwdOutput = new FWDOutput();
		strOutput = new STROutput();
		strVision = new STRVision();

		pidDistance = new PIDController(0.02, 0, 0, pidVision, fwdOutput);
		pidStrafe = new PIDController(0.02, 0, 0, strVision, strOutput);

		Calibration.loadSwerveCalibration();

		line = new AnalogInput(0);

		compressor = new Compressor(0);
		compressor.setClosedLoopControl(true);

		DriveTrain.setDrivePIDValues(Calibration.AUTO_DRIVE_P, Calibration.AUTO_DRIVE_I, Calibration.AUTO_DRIVE_D);

		RobotGyro.reset(); // this is also done in auto init in case it wasn't
							// settled here yet

		SmartDashboard.putBoolean("Show Turn Encoders", true);
		SmartDashboard.putNumber("Vision P", kVisionP);
		SmartDashboard.putNumber("Vision D", kVisionD);

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
		
		SmartDashboard.putNumber("Version", 2.8);
		
		double newP = SmartDashboard.getNumber("Vision P", 0);
		double newD = SmartDashboard.getNumber("Vision D", 0);

		
		// if (newP != kVisionP) {
		// 	kVisionP = newP;
		// 	pidDrive.setP(kVisionP);
		// }
		// if (newD != kVisionD) {
		// 	kVisionD = newD;
		// 	pidDrive.setD(kVisionD);
		// }

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

		Vision.tick();

		// A
		if (gamepad.getHID(0).getRawButton(1)) {
			
			inAutoMode = true;
	
			Vision.setTargetTrackingMode();
			pidDistance.setSetpoint(72);
			pidDistance.enable();
			pidStrafe.setSetpoint(0);
			pidStrafe.enable();
	
		}
		// B
		if (gamepad.getHID(0).getRawButton(2)) {
			Vision.setLED(false);
			Vision.setDriverMode();

			pidDistance.disable();
			pidStrafe.disable();
	
			inAutoMode = false;
		}

		if (inAutoMode) {
			double fwd = fwdOutput.getValue();
			double str = strOutput.getValue();

			if(Vision.targetInfoIsValid()){
				// pidDrive.enable();
				pidDistance.enable();
				pidStrafe.enable();
			} else {
				// pidDrive.disable();
				pidDistance.disable();
				pidStrafe.disable();
				fwd = 0;
				str = 0;
			}
			
			DriveTrain.swerveDrive(fwd, str, 0);

			SmartDashboard.putNumber("fwd", fwd);
			SmartDashboard.putNumber("str", str);

		} else {
			// DRIVER CONTROL MODE
			// Issue the drive command using the parameters from
			// above that have been tweaked as needed
	
			DriveTrain.fieldCentricDrive(driveYAxisAmount, driveXAxisAmount, driveRotAxisAmount);
		}


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

		// in case we're tweaking the PID via Dashboard, update the values
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
