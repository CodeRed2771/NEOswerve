package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoEpicStart extends AutoBaseClass {

    AutoBaseClass mSubAutoProg = new AutoDoNothing();
    
    public static enum robotPosition{
        LEFT, CENTER, RIGHT
    };


    public void tick (){
        if (isRunning()) {

            DriveAuto.tick();

            switch(getCurrentStep()){

                    case 0:
                    if (robotPosition () == Position.LEFT){
                        driveInches(30, 270, 1);
                    }
                        else if (robotPosition () == Position.CENTER){
                            driveInches(40, 0, 1);
                        }
                            else if (robotPosition() == Position.RIGHT){
                                driveInches(30, 90, 1);
                            }
                        setTimerAndAdvanceStep(2000);
                        break;

                        case 1:
                        if (driveCompleted()){
                            advanceStep();
                            break;
                        }
                        case 2:
                        mSubAutoProg = new AutoDoEverything();
                        mSubAutoProg.start();
                        advanceStep();
                        break;
                    
                    
                    }

                
            }


        }


    
}



