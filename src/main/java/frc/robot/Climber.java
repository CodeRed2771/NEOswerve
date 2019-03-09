package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber {
  private static Climber instance;
  private static CANSparkMax climbMotor = new CANSparkMax(Wiring.CLIMB_MOTOR, MotorType.kBrushless);

  public static final double EXTENDED_POSITION = 274;

  public Climber() {
    climbMotor.getPIDController().setOutputRange(-0.85, 0.85);
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

  public static void climbManual(double speed) {
    climbMotor.set(speed);
  }

  public static void climberExtend() {
    climbMotor.getPIDController().setReference(EXTENDED_POSITION, ControlType.kPosition);
  }

  public static void climberRetract() {
    climbMotor.getPIDController().setReference(0, ControlType.kPosition);
  }

  public static boolean isExtended() {
    // if we're within 15 ticks of the target position, consider it extended.
    return climbMotor.getEncoder().getPosition() >= (EXTENDED_POSITION - 15);
  }

  public static void manualDrive(double speed) {
    climbMotor.set(speed);
    SmartDashboard.putNumber("ModB Enc", DriveTrain.moduleB.getDriveEnc());
  }

  public static void moveSetpoint(double direction) {
    double newSetpoint;

    if (direction > 0) {
      newSetpoint = climbMotor.getEncoder().getPosition() + 39.05;
      // if (newSetpoint >= 0) {
      // newSetpoint = 0;
      // }
    } else {
      newSetpoint = climbMotor.getEncoder().getPosition() - 39.05;
      // if (newSetpoint < -40000) {
      // newSetpoint = -40000;
      // }
    }

    climbMotor.getPIDController().setReference(newSetpoint, ControlType.kPosition);
  }
}
