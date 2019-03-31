package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;

public class RobotGyro implements PIDSource {
	private static RobotGyro instance;
	private static AHRS mGyro;

	public static RobotGyro getInstance() {
		if (instance == null) {
			try {
				instance = new RobotGyro();
			} catch (Exception ex) {
				System.out.println("RobotGyro could not be initialized due to the following error: ");
				System.out.println(ex.getMessage());
				System.out.println(ex.getStackTrace());
				return null;
			}
		}

		return instance;
	}

	public RobotGyro() {
		// mGyro = new AHRS(SerialPort.Port.kUSB);
		// mGyro = new AHRS(I2C.Port.kMXP); // i2c is 10x slower than SPI
		mGyro = new AHRS(SPI.Port.kMXP);
	}

	public static AHRS getGyro() {
		if (getInstance() == null)
			return null;

		return mGyro;
	}

	/***
	 * 
	 * @return gyro angle This angle can include multiple revolutions so if the
	 *         robot has rotated 3 1/2 times, it will return a value of 1260. This
	 *         is useful for turning but not as useful for figuring out which
	 *         direction you're facing. Use getRelativeAngle for that.
	 */
	public static double getAngle() {
		if (getInstance() == null)
			return 0.0;

		return mGyro.getAngle();
	}

	/***
	 * 
	 * @return gyro angle 0 to 360
	 * unless you allow negatives, then it's -360 to +360
	 */
	public static double getRelativeAngle(boolean allowNegatives) {
		if (getAngle() < 0 && !allowNegatives)
			return 360 + (getAngle() % 360);
		else
			return getAngle() % 360;

	}

	public static void reset() {
		if (getInstance() == null)
			return;

		mGyro.reset();
	}

	public static double getGyroAngleInRad() {
		if (getInstance() == null)
			return 0.0;

		double adjustedAngle = -Math.floorMod((long) mGyro.getAngle(), 360);
		if (adjustedAngle > 180)
			adjustedAngle = -(360 - adjustedAngle);

		return adjustedAngle * (Math.PI / 180d);
	}

	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		mGyro.setPIDSourceType(pidSource);
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		return mGyro.getPIDSourceType();
	}

	public double pidGet() {
		return mGyro.getAngle();
	}

}
