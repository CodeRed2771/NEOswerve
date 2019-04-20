package frc.robot;

//This is definitely not authorized by DVV.
public class AutoTurn extends AutoBaseClass {
    public void tick() {
        final int rightRocketAngle = 150;
        final int leftRocketAngle = 330;
        double currentGyroAngle = RobotGyro.getRelativeAngle();

        if (isRunning()) {

            DriveAuto.tick();
            System.out.println("TRYING TO RUN");

            switch (getCurrentStep()) {
            case 0:
                if (currentGyroAngle < 175 && currentGyroAngle > 5) {
                    double turnAmount = rightRocketAngle - currentGyroAngle;
                    turnDegrees(turnAmount, 1);
                    System.out.println("Calling for turn on right rocket " + turnAmount);
                } else if (currentGyroAngle > 185 && currentGyroAngle < 355) {
                    double turnAmount = leftRocketAngle - currentGyroAngle;
                    turnDegrees(turnAmount, 1);
                    System.out.println("Calling for turn on left rocket " + turnAmount);
                }
                setTimerAndAdvanceStep(3000);
                break;
            case 1:
                if (turnCompleted()) {
                    advanceStep();
                }
                break;
            case 2:
                stop();
                break;
            }
        }
    }
}