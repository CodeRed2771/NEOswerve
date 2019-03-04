package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber {
    private static Climber instance;
    private static CANSparkMax climbMotor = new CANSparkMax(Wiring.CLIMB_MOTOR, MotorType.kBrushless);

    public Climber() {
      climbMotor.getPIDController().setOutputRange(-0.25, 0.25);
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
    
    public static void climb(double speed) {  
      climbMotor.set(speed);
    }
    public static void moveSetpoint(double direction) {
      double newSetpoint;
  
      if (direction > 0) {
        newSetpoint = climbMotor.getEncoder().getPosition() + 39.05;
        // if (newSetpoint >= 0) {
        //   newSetpoint = 0;
        // }
      } else {
        newSetpoint = climbMotor.getEncoder().getPosition() - 39.05;
        // if (newSetpoint < -40000) {
        //   newSetpoint = -40000;
        // }
      }
  
      climbMotor.getPIDController().setReference(newSetpoint, ControlType.kPosition);
    }

}
