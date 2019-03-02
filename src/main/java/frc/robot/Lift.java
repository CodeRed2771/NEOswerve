package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Lift {
	private static Lift instance;
	private static TalonSRX liftMotor;
	// private static TalonSRX liftFollower;
	
	public static Lift getInstance() {
		if (instance == null)
			instance = new Lift();
		return instance;
	}

	public Lift() {
		liftMotor = new TalonSRX(Wiring.LIFT_MASTER);
		// liftFollower = new TalonSRX(Wiring.LIFT_FOLLLOWER);
		// liftFollower.follow(liftMotor);
		// liftFollower.setInverted(false);
		
		/* first choose the sensor */
		liftMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,0, 0);
		
		/* set the relevant frame periods to be at least as fast as periodic rate */
		liftMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10,0);
		liftMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10,0);

		/* set the peak and nominal outputs*/
		liftMotor.configNominalOutputForward(0, 0);
		liftMotor.configNominalOutputReverse(0, 0);
		liftMotor.configPeakOutputForward(1, 0);
		liftMotor.configPeakOutputReverse(-1, 0);
		liftMotor.setNeutralMode(NeutralMode.Brake);
		
		liftMotor.configClosedloopRamp(.25, 0);
		
		/*set closed loop gains in slot0 - see documentation*/
		liftMotor.selectProfileSlot(0, 0);
		liftMotor.config_kF(0, 1, 0);
		liftMotor.config_kP(0, .6, 0);
		liftMotor.config_kI(0, 0, 0);
		liftMotor.config_kD(0, 0, 0);
		
		SmartDashboard.putNumber("MM Lift Velocity", 4000);
		SmartDashboard.putNumber("MM Lift Acceleration", 2000);
		SmartDashboard.putNumber("Lift F", 1);
		SmartDashboard.putNumber("Lift P", .6);
		SmartDashboard.putNumber("Lift I", 0);
		SmartDashboard.putNumber("Lif D", 0);
		
		/* set acceleration and vcruise velocity - see documentation*/
		liftMotor.configMotionCruiseVelocity(4000, 0);
		liftMotor.configMotionAcceleration(2000, 0);
		
		/* zero the sensor */
		liftMotor.setSelectedSensorPosition(0, 0, 0);
	}

	public static void tick() {
		
		liftMotor.configMotionCruiseVelocity((int)SmartDashboard.getNumber("MM Lift Velocity", 0), 0);
		liftMotor.configMotionAcceleration((int)SmartDashboard.getNumber("MM Lift Acceleration", 0), 0);
		liftMotor.config_kF(0, SmartDashboard.getNumber("Lift F", 1.0), 0);
		liftMotor.config_kP(0, SmartDashboard.getNumber("Lift P", 1.0), 0);
		liftMotor.config_kI(0, SmartDashboard.getNumber("Lift I", 0), 0);
		liftMotor.config_kD(0, SmartDashboard.getNumber("Lift D", 0), 0);
		SmartDashboard.putNumber("Lift Encoder", liftMotor.getSensorCollection().getQuadraturePosition());
		SmartDashboard.putNumber("Lift Setpoint", liftMotor.getClosedLoopTarget());
	}
	public static  void move(double speed) {
		// limit speed
		if (speed < -.3) 
			speed = -.3;
		if (speed > .3) 
			speed = .3;

		liftMotor.set(ControlMode.PercentOutput, speed);	
	}

	public static void moveSetpoint(double direction) {
		int newSetpoint;

		if (direction > 0) {
			newSetpoint = liftMotor.getSelectedSensorPosition(0) + 4000;
			if (newSetpoint >= 0) {
				newSetpoint = 0;
			}
		} else {
			newSetpoint = liftMotor.getSelectedSensorPosition(0) - 4000;
			if (newSetpoint < -40000) {
				newSetpoint = -40000;
			}
		}

		liftMotor.set(ControlMode.MotionMagic, newSetpoint);
	}

	public static void goToStart(){
		liftMotor.set(ControlMode.MotionMagic, 0);
	}
	public static  void goHatchLvl1() {
		liftMotor.set(ControlMode.MotionMagic, -1340); 
	}
	public static void goCargoLvl1() {
		liftMotor.set(ControlMode.MotionMagic, -4340);
	}
	public static  void goHatchLvl2() {
		liftMotor.set(ControlMode.MotionMagic, -11000); 
	}
	public static void goCargoLvl2() {
		liftMotor.set(ControlMode.MotionMagic, -14000); 
    }
    public static void goHatchLvl3(){
        liftMotor.set(ControlMode.MotionMagic, -22000); 
    }
	public static void goCargoLvl3(){
		liftMotor.set(ControlMode.MotionMagic, -25000); 
	}
    public static void goCargoShipCargo(){
        liftMotor.set(ControlMode.MotionMagic, -4300); 
    }
	public static void stop(){
		liftMotor.set(ControlMode.PercentOutput, 0);	
	}
	
	// returns true if the lift is high enough that we should reduce drving speed
	public static boolean driveCautionNeeded() {
		return Math.abs(liftMotor.getSensorCollection().getPulseWidthPosition()) > 15000; 
	}
}
