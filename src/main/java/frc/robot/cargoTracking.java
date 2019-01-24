// package frc.robot;

// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.networktables.NetworkTable;

// public class cargoTracking extends AutoBaseClass {
// 	NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("limelight");
// 	public cargoTracking(int robotPosition){
// 		networkTable.getEntry("ledMode").forceSetNumber(1);
// 		networkTable.getEntry("pipeline").forceSetNumber(1);
// 	}
	
// 	public void tick(){
// 		if(isRunning()){
// 			switch(getCurrentStep()){
// 			case 0:
// 				setTimerAndAdvanceStep(2000);
// 				driveInches(200,0,.5);
// 				break;
// 			case 1:
// 				if(driveCompleted())
// 					advanceStep();
// 				break;
// 			case 2:
// 				setTimerAndAdvanceStep(2000);
// 				driveInches(80,0,.5);
// 				break;
// 			case 3:
// 				if(driveCompleted())
// 					advanceStep();
// 				break;
// 			case 10:
// 				if(networkTable.getEntry("tv").getNumber(0).intValue() == 0){
					
// 				}
// 			}
// 		}
// 	}
// }