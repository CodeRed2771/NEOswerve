/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.libs.CurrentBreaker;

public class CargoPickup {
    private static CargoPickup instance;
    private static TalonSRX cargoPickup;

    private static CurrentBreaker currentBreaker1;
    private static CurrentBreaker currentBreaker2;

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
            cargoPickup = new TalonSRX(Wiring.CUBE_CLAW_LEFT_MOTOR);
            cargoPickup.setInverted(true);
    
            cargoPickup.configOpenloopRamp(.2, 0);
    
            cargoPickup.setNeutralMode(NeutralMode.Brake);
    
            currentBreaker1 = new CurrentBreaker(null, Wiring.CLAW_PDP_PORT1, Calibration.CLAW_MAX_CURRENT, 250, 2000); 
            currentBreaker2 = new CurrentBreaker(null, Wiring.CLAW_PDP_PORT2, Calibration.CLAW_MAX_CURRENT, 250, 2000);

            resetIntakeStallDetector();
    
            ejectEndTime = aDistantFutureTime();
            startReverseTime = aDistantFutureTime();
    
        }

    /*
     * TICK -----------------------------------------------
     */
    public static void tick() {
        SmartDashboard.putNumber("Intake Current 1", currentBreaker1.getCurrent());
        SmartDashboard.putNumber("Intake Current 2", currentBreaker2.getCurrent());

        if ((currentBreaker1.getCurrent() > 15) || (currentBreaker2.getCurrent() > 15)) {
            reverseAllowed = true;
        } else
            reverseAllowed = false;

        if (intakeStalled() && !holdingCargo) {
            System.out.println("Intake stalled - switching to hold mode");
            holdCargo();
        }

        // this turns off the claw after starting an eject
        if (System.currentTimeMillis() > ejectEndTime) {
            System.out.println("Stopped ejecting");
            stopIntake();
            ejectEndTime = aDistantFutureTime();
        }

        if (intakeRunning) {
            if (System.currentTimeMillis() >= startReverseTime) {
                if (reverseAllowed)
                    reverseIntake();
            }
            if (System.currentTimeMillis() >= (startReverseTime + 200)) {
                intakeCargo();
            }
        }

    }

    // CONTROL METHODS ------------------------------------------------

    public static void intakeCargo() {
        holdingCargo = false;
        cargoPickup.set(ControlMode.PercentOutput, -.8);
        resetIntakeStallDetector();
        ejectEndTime = aDistantFutureTime();
        intakeRunning = true;
        startReverseTime = System.currentTimeMillis() + 200;
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

    public static void dropCargo() {
        holdingCargo = false;
        resetIntakeStallDetector();
    }

    public static void ejectCargo() {
        holdingCargo = false;
        resetIntakeStallDetector();
        cargoPickup.set(ControlMode.PercentOutput, .5);

        ejectEndTime = System.currentTimeMillis() + 750;
    }

    public static void stopIntake() {
        cargoPickup.set(ControlMode.PercentOutput, 0);
        resetIntakeStallDetector();
        intakeRunning = false;
    }

    // UTILITY METHODS ---------------------------------------------------------

    public static boolean intakeStalled() {
        return (currentBreaker1.tripped() || currentBreaker2.tripped());
    }

    public static void resetIntakeStallDetector() {
        currentBreaker1.reset();
        currentBreaker2.reset();
    }

    /*
     * Resets the arm encoder value relative to what we've determined to be the
     * "zero" position. (the calibration values). This is so the rest of the program
     * can just treat the turn encoder as if zero is the horizontal position. We
     * don't have to always calculate based off the calibrated zero position. e.g.
     * if the calibrated zero position is .25 and our current absolute position is
     * .40 then we reset the encoder value to be .15 * 4095, so we know were .15
     * away from the zero position. The 4095 converts the position back to ticks.
     * 
     * Bottom line is that this is what applies the turn calibration values.
     */

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
