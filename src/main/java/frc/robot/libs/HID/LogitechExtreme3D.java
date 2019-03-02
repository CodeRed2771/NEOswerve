package frc.robot.libs.HID;

/**
 *
 * @author Austin
 */
public class LogitechExtreme3D extends HID {

    public static final Axis STICK_X = new Axis(0, 0.05);
    public static final Axis STICK_Y = new Axis(1, 0.05, -1);
    public static final Axis STICK_ROT = new Axis(2, 0.05);
   
    /**
     *
     * @param port The port (1-4) that the controller is connected to in the
     * Driver Station.
     */
    public LogitechExtreme3D(int port) {
        super(port);
    }
}