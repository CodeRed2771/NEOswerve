package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber {
    private static Climber instance;
    private static CANSparkMax climbMotor = new CANSparkMax(Wiring.CLIMB_MOTOR, MotorType.kBrushless);
    private static TalonSRX climbDrive = new TalonSRX(Wiring.CLIMB_DRIVE);

    public static final double EXTENDED_POSITION = 284;

    public Climber() {
        climbMotor.getPIDController().setOutputRange(-0.90, 0.90);
        climbMotor.getPIDController().setP(1);
    }

    public static Climber getInstance() {
        if (instance == null)
            instance = new Climber();
        return instance;
    }

    public static void tick() {
        SmartDashboard.putNumber("Climb Enc", climbMotor.getEncoder().getPosition());
    }

    public static void stop() {
        climbDrive.set(ControlMode.Velocity,0);
        climbMotor.set(0);
    }
    public static void setClimbDriveSpeed(double speed) {
        climbDrive.set(ControlMode.PercentOutput, speed);
    }

    public static void climbManual(double speed) {
        climbMotor.set(speed);
    }

    public static void climberExtend() {
        climbMotor.getPIDController().setReference(EXTENDED_POSITION, ControlType.kPosition);
    }

    public static void climberRetract() {
        climbMotor.getPIDController().setReference(EXTENDED_POSITION - (EXTENDED_POSITION / 3), ControlType.kPosition);
    }

    public static boolean isExtended() {
        // if we're within 15 ticks of the target position, consider it extended.
        return climbMotor.getEncoder().getPosition() >= (EXTENDED_POSITION - 15);
    }

    public static void moveSetpoint(double direction) {
        double newSetpoint;

        if (direction > 0) {
            newSetpoint = climbMotor.getEncoder().getPosition() + (39.05 / 2); // half revolution
            // if (newSetpoint >= 0) {
            // newSetpoint = 0;
            // }
        } else {
            newSetpoint = climbMotor.getEncoder().getPosition() - (39.05 / 2); // half rotation
            // if (newSetpoint < -40000) {
            // newSetpoint = -40000;
            // }
        }

        climbMotor.getPIDController().setReference(newSetpoint, ControlType.kPosition);
    }
}
