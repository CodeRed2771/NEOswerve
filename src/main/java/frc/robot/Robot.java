
package frc.robot;

import java.math.BigDecimal;
import java.math.RoundingMode;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/* 2018 Xena - PowerUp

 	Controls
		DRIVER
			Left Stick moves robot in X Y plane
			Right Stick rotates robot
			A - puts arm in horizontal position
			B - puts arm in travel (up) position
			X - resets the arm encoder
			Start - puts lift in high speed
			Back  - puts lift in low gear
			
		MANIPULATOR
			A - lift to bottom position, arm horizontal
			B - lift to switch position, arm in switch pos
			Y - lift to scale position, arm in scale pos
			X - arm over the top
			Start - arm in travel (up) position
			Dpad Left (DL) - arm modifier
				DL A - portal height, arm level
				DL B - second cube height, arm level
			Dpad Right (DR) - lift modifier
				DR A - lift to low scale
				DR B - lift to med scale
				DR Y - lift to high scale
			Right Bumper - Start Intake and lower arm
			Left Bumper - Drop cube
			Back - eject cube
			Right Stick Y - manual arm
			Left Stick Y - manual lift
			Right Trigger - force open claw during intake
			
*/
public class Robot extends TimedRobot {

	KeyMap gamepad;
	Compressor compressor;
	SendableChooser<String> autoChooser;
	SendableChooser<String> positionChooser;
	AnalogInput line;

	@Override
	public void robotInit() {
		gamepad = new KeyMap();
		RobotGyro.getInstance();
		DriveTrain.getInstance();
		DriveAuto.getInstance();

		Calibration.loadSwerveCalibration();

		line = new AnalogInput(0);
	
		compressor = new Compressor(0);
		compressor.setClosedLoopControl(true);

		DriveTrain.setDrivePIDValues(Calibration.AUTO_DRIVE_P, Calibration.AUTO_DRIVE_I,
				Calibration.AUTO_DRIVE_D);

		RobotGyro.reset(); // this is also done in auto init in case it wasn't
							// settled here yet

		SmartDashboard.putBoolean("Show Turn Encoders", true);
		
		SmartDashboard.putNumber("line sensor", line.getAverageValue());
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

		SmartDashboard.putNumber("Match Time", DriverStation.getInstance().getMatchTime());
		
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
		
		// Issue the drive command using the parameters from
		// above that have been tweaked as needed
		
		// DriveTrain.fieldCentricDrive(driveYAxisAmount, driveXAxisAmount,
		// 		driveRotAxisAmount);
				DriveTrain.fieldCentricDrive(0, 0,	0);
	

		SmartDashboard.putNumber("Gyro Heading", round0(RobotGyro.getAngle()));

		if (SmartDashboard.getBoolean("Show Turn Encoders", false)) {
			DriveTrain.showTurnEncodersOnDash();
			DriveTrain.showDriveEncodersOnDash(); 
		}
		
		DriveTrain.setTurnPIDValues(SmartDashboard.getNumber("TURN P", Calibration.TURN_P),
				SmartDashboard.getNumber("TURN I", Calibration.TURN_I),
				SmartDashboard.getNumber("TURN D", Calibration.TURN_D));

	}

	
	@Override
	public void autonomousInit() {
		

	}

	@Override
	public void autonomousPeriodic() {


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

	private double powerOf2PreserveSign(double v) {
		return (v > 0) ? Math.pow(v, 2) : -Math.pow(v, 2);
	}

	private double powerOf3PreserveSign(double v) {
		return Math.pow(v, 3);
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
