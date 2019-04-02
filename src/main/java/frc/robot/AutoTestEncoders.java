package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*
 * This auto program is a rotation
 * calibration test.
 */

public class AutoTestEncoders extends AutoBaseClass {
    
    private long startTime = -1;

    private double moduleADriveInitial;
    private double moduleBDriveInitial;
    private double moduleCDriveInitial;
    private double moduleDDriveInitial;

    @Override
    public void start() {
        super.start(); 

        moduleADriveInitial = DriveTrain.moduleA.getDriveEnc();
        moduleBDriveInitial = DriveTrain.moduleB.getDriveEnc();
        moduleCDriveInitial = DriveTrain.moduleC.getDriveEnc();
        moduleDDriveInitial = DriveTrain.moduleD.getDriveEnc();
        
        // Shuffleboard.getTab("Encoders").
        SmartDashboard.putBoolean("Encoders/moduleADrive", true);
        SmartDashboard.putBoolean("Encoders/moduleBDrive", true);
        SmartDashboard.putBoolean("Encoders/moduleCDrive", true);
        SmartDashboard.putBoolean("Encoders/moduleDDrive", true);
        SmartDashboard.putBoolean("Encoders/moduleATurn", true);
        SmartDashboard.putBoolean("Encoders/moduleBTurn", true);
        SmartDashboard.putBoolean("Encoders/moduleCTurn", true);
        SmartDashboard.putBoolean("Encoders/moduleDTurn", true);
        SmartDashboard.putBoolean("Encoders/lift", true);
        SmartDashboard.putBoolean("Encoders/linkage", true);
        SmartDashboard.putBoolean("Encoders/selfTestCompleted", false);

        DriveAuto.driveInches(10, 0, 0.5);
        DriveTrain.setAllTurnOrientation(0, true);

        startTime = System.currentTimeMillis();
    }

    @Override
	public void tick() {
		if (isRunning()) {
            if (System.currentTimeMillis() > startTime + 2000) {
                if (isAboutTheSame(DriveTrain.moduleA.getDriveEnc(), moduleADriveInitial)) {
                    SmartDashboard.putBoolean("Encoders/moduleADrive", false);
                    System.err.println("There may be an issue with the module A drive encoder.");
                }
                if (isAboutTheSame(DriveTrain.moduleB.getDriveEnc(), moduleBDriveInitial)) {
                    SmartDashboard.putBoolean("Encoders/moduleBDrive", false);
                    System.err.println("There may be an issue with the module B drive encoder.");
                }
                if (isAboutTheSame(DriveTrain.moduleA.getDriveEnc(), moduleCDriveInitial)) {
                    SmartDashboard.putBoolean("Encoders/moduleCDrive", false);
                    System.err.println("There may be an issue with the module C drive encoder.");
                }
                if (isAboutTheSame(DriveTrain.moduleA.getDriveEnc(), moduleDDriveInitial)) {
                    SmartDashboard.putBoolean("Encoders/moduleDDrive", false);
                    System.err.println("There may be an issue with the module D drive encoder.");
                }
                if (!isAboutTheSame(DriveTrain.moduleA.getTurnAbsolutePosition(), Calibration.GET_DT_A_ABS_ZERO())) {
                    SmartDashboard.putBoolean("Encoders/moduleATurn", false);
                    System.err.println("There may be an issue with the module A turn encoder.");
                }
                if (!isAboutTheSame(DriveTrain.moduleB.getTurnAbsolutePosition(), Calibration.GET_DT_B_ABS_ZERO())) {
                    SmartDashboard.putBoolean("Encoders/moduleBTurn", false);
                    System.err.println("There may be an issue with the module B turn encoder.");
                }
                if (!isAboutTheSame(DriveTrain.moduleC.getTurnAbsolutePosition(), Calibration.GET_DT_C_ABS_ZERO())) {
                    SmartDashboard.putBoolean("Encoders/moduleCTurn", false);
                    System.err.println("There may be an issue with the module C turn encoder.");
                }
                if (!isAboutTheSame(DriveTrain.moduleD.getTurnAbsolutePosition(), Calibration.GET_DT_D_ABS_ZERO())) {
                    SmartDashboard.putBoolean("Encoders/moduleDTurn", false);
                    System.err.println("There may be an issue with the module D turn encoder.");
                }
                if (Lift.getEncoderPosition() == 0) {
                    SmartDashboard.putBoolean("Encoders/lift", false);
                    System.err.println("There may be an issue with the lift encoder.");
                }
                if (Manipulator.getLinkageEncoderPosition() == 0) {
                    SmartDashboard.putBoolean("Encoders/linkage", false);
                    System.err.println("There may be an issue with the linkage encoder.");
                }
                SmartDashboard.putBoolean("Encoders/selfTestCompleted", true);
                stop();
            }
        }
    }
    
    private boolean isAboutTheSame(double valueA, double valueB) {
        return Math.abs(valueA - valueB) < 50.0;
    }
}
