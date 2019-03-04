/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.libs.CurrentBreaker;

public class CargoPickup {
    private static CargoPickup instance;
    private static TalonSRX cargoPickup;
    private static CurrentBreaker currentBreaker;

    private static boolean holdingCargo = false;
    private static boolean intakeRunning = false;
    private static double ejectEndTime;
    private static double startReverseTime;
    private static boolean reverseAllowed = false;

    public static CargoPickup getInstance() {
        if (instance == null)
            instance = new CargoPickup();
        return instance;
    }

    public CargoPickup() {
            cargoPickup = new TalonSRX(Wiring.INTAKE_MOTOR);
            cargoPickup.setInverted(true);
    
            cargoPickup.configOpenloopRamp(.2, 0);
    
            cargoPickup.setNeutralMode(NeutralMode.Brake);
    
            currentBreaker = new CurrentBreaker(null, Wiring.INTAKE_PDP_PORT, Calibration.INTAKE_MAX_CURRENT, 250, 2000); 
  
            resetIntakeStallDetector();
    
            ejectEndTime = aDistantFutureTime();
            startReverseTime = aDistantFutureTime();
    
        }

    /*
     * TICK -----------------------------------------------
     */
    public static void tick() {
        
        SmartDashboard.putNumber("Intake Current", currentBreaker.getCurrent());
  
        if (intakeStalled() && !holdingCargo) {
            System.out.println("Intake stalled - switching to hold mode");
            holdCargo();
        }

        // this turns off the intake a little while after starting an eject
        if (System.currentTimeMillis() > ejectEndTime) {
            System.out.println("Stopped ejecting");
            stopIntake();
            ejectEndTime = aDistantFutureTime();
        }
    }

    // CONTROL METHODS ------------------------------------------------

    public static void intakeCargo() {

        cargoPickup.set(ControlMode.PercentOutput, -.8);

        intakeRunning = true;
        holdingCargo = false;
        resetIntakeStallDetector();
        ejectEndTime = aDistantFutureTime();
    }

    public static void reverseIntake() {
        cargoPickup.set(ControlMode.PercentOutput, .4);
    }

    public static boolean isIntakeRunning() {
        return intakeRunning;
    }

    public static void holdCargo() {
        stopIntake();
        cargoPickup.set(ControlMode.PercentOutput, -.15);
        holdingCargo = true;
    }

    public static void ejectCargo() {

        cargoPickup.set(ControlMode.PercentOutput, .5);

        holdingCargo = false;
        resetIntakeStallDetector();

        ejectEndTime = System.currentTimeMillis() + 750;
    }

    public static void stopIntake() {
        cargoPickup.set(ControlMode.PercentOutput, 0);
        resetIntakeStallDetector();
        intakeRunning = false;
    }

    // UTILITY METHODS ---------------------------------------------------------

    public static boolean intakeStalled() {
        return (currentBreaker.tripped());
    }

    public static void resetIntakeStallDetector() {
        currentBreaker.reset();
    }

    private static double aDistantFutureTime() {
        return System.currentTimeMillis() + 900000; // 15 minutes in the future
    }

    /*
     * TEST METHODS
     */

    public static void testIntakeCargo(double speed) {
        cargoPickup.set(ControlMode.PercentOutput, speed);
    }

    public static void testEjectCargo(double speed) {
        cargoPickup.set(ControlMode.PercentOutput, -speed);
    }

}
