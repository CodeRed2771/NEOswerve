package frc.robot.libs;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 * @author Michael
 * 3/9/16 - revised by D. Van Voorst
 * This class can be used to simplify timing autonomous steps
 * Use getStep() to manage your state machine
 * Use setTimerAndAdvanceStep() to start a timer for a particular step
 * Use advanceWhenReady() to automatically advance to the next auto step when the timer is reached
 * Use stopTimeAndAdvanceStep() when your drive command has reached its target to prevent the advanceWhenReady from firing also
 * 
 */
public class Timer {

    private int step;
    private long endTime;
    private boolean timerRunning = false;
    
    public Timer() {

    }

    public void setTimer(long time) {
    	endTime = System.currentTimeMillis() + time;
    	timerRunning = true;
    }
    
    public void setTimerAndAdvanceStep(long time) {
    	setTimer(time);
    	nextStep();
    }
    
    public void stopTimerAndAdvanceStep() {
    	setTimer(1000000); // push the target time way into the future so it never reaches the target time
    	nextStep();
    }
    
    public void advanceWhenTimerExpired() {
        if (timeExpired() && timerRunning) {
            step++;
            timerRunning = false;
        }
    }
    
    public boolean timeExpired() {
        return endTime < System.currentTimeMillis();
    }

    public int getStep() {
        return step;
    }

    public void setStep(int step) {
        this.step = step;
    }
    
    public void nextStep() {
        this.step++;
    }
    
    public double getTimeRemainingMilliseconds() {
    	return endTime - System.currentTimeMillis();
    }
    
    public double getTimeRemainingSeconds() {
    	return (endTime - System.currentTimeMillis())/1000;
    }
    
    public void tick() {
    	
    	advanceWhenTimerExpired();
    	
    	SmartDashboard.putNumber("Timer Step", step);
    	SmartDashboard.putBoolean("Timer Is Running", timerRunning);
    	SmartDashboard.putNumber("Timer Remaining MS",  getTimeRemainingSeconds());
    }
}
