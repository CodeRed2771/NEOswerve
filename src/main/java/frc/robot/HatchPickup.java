/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class HatchPickup {
    private static DoubleSolenoid fingers;
    public static void hatchPickup() {
        fingers = new DoubleSolenoid(Wiring.CLAW_PCM_PORTA, Wiring.CLAW_PCM_PORTB);
    }
    private static void turnOffHatchPickup() {
		fingers.set(DoubleSolenoid.Value.kOff);
    }
    private static void openHatchPickup() {
		fingers.set(DoubleSolenoid.Value.kForward);
    }
    private static void closeHatchPickup() {
		fingers.set(DoubleSolenoid.Value.kReverse);
	}
}
