/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Add your docs here.
 */
public class TargetInfo {
    public static enum TargetType {
        SHIP_TARGET, ROCKET_TARGET, FEEDER_TARGET,
    }

    private static TargetInfo instance;

    public static TargetInfo getInstance() {
        if (instance == null) {
            try {
                instance = new TargetInfo();
            } catch (Exception ex) {
                System.out.println("Target Info module could not be initialized due to the following error: ");
                System.out.println(ex.getMessage());
                System.out.println(ex.getStackTrace());
                return null;
            }
        }
        return instance;
    }

    public static double targetAngle(TargetType targetType) {
        double currentGyroAngle = RobotGyro.getAngle();

        if (targetType == TargetType.ROCKET_TARGET) {
            if (Manipulator.isHoldingCargo()) {
                if (currentGyroAngle > -10 && currentGyroAngle < -170) {
                    System.out.println("Holding Cargo, returning -90, current gyro angle: " + currentGyroAngle);
                    return -90;
                } else {
                    System.out.println("Holding Cargo, returning 90, current gyro angle: " + currentGyroAngle);
                    return 90;
                }
            } else {
                if (currentGyroAngle > -80 && currentGyroAngle < -10){
                    System.out.println("Holding Hatch, returning -30, current gyro angle: " + currentGyroAngle);
                    return -30;
                } 
                else if (currentGyroAngle > -170 && currentGyroAngle < -100){
                    System.out.println("Holding Hatch, returning -150, current gyro angle: " + currentGyroAngle);
                    return -150;
                }
                    
                else if (currentGyroAngle > 100 && currentGyroAngle < 170){
                    System.out.println("Holding Hatch, returning 150, current gyro angle: " + currentGyroAngle);
                    return 150;
                }

                else {
                    System.out.println("Holding Hatch, returning 30, current gyro angle: " + currentGyroAngle);
                    return 30;
                }
            }
        }
        else if (targetType == TargetType.SHIP_TARGET) {
            if (currentGyroAngle > -30 && currentGyroAngle < -170){
                System.out.println("Cargo Ship, Returning -90, current gyro angle: " + currentGyroAngle);
                return -90;
            }
            else if (currentGyroAngle > 30 && currentGyroAngle < 170) {
                System.out.println("Cargo Ship, Returning 90, current gyro angle: " + currentGyroAngle);
                return 90;
            }
            else { 
                System.out.println("Cargo Ship, Returning 0, current gyro angle: " + currentGyroAngle);
                return 0;
            }   

        } else {
            System.out.println("Feeder Station, Returning 180, current gyro angle: " + currentGyroAngle);
            return 180;
        }


        // if (targetType == TargetType.ROCKET_TARGET) {
        // if (currentGyroAngle > 6 && currentGyroAngle < 75) {
        // return 30;
        // }
        // if (currentGyroAngle > 76 && currentGyroAngle < 170) {
        // return 90;
        // }
        // if (currentGyroAngle < -10 && currentGyroAngle > -90) {
        // return -30;
        // }
        // if (currentGyroAngle > -6 && currentGyroAngle < -75) {
        // return -30;
        // }
        // if (currentGyroAngle > -76 && currentGyroAngle < -170) {
        // return -90;
        // }
        // if (currentGyroAngle < -10 && currentGyroAngle > -90) {
        // return -150;
        // }
        // }
        // if (targetType == TargetType.SHIP_TARGET) {
        // if (currentGyroAngle < 40 && currentGyroAngle > -40) {
        // return 0;
        // }
        // if (currentGyroAngle < 70 && currentGyroAngle > 120) {
        // return 90;
        // }
        // if (currentGyroAngle < -70 && currentGyroAngle > -120) {
        // return -90;
        // }
        // }
        // if (targetType == TargetType.FEEDER_TARGET) {
        // return 180;
        // }

        // return 90;

        // // NEED MORE IF STATEMENTS FOR OTHER POSITIONS

        // return 90;

    }
}
