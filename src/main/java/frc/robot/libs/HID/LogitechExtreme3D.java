package frc.robot.libs.HID;

/**
 *
 * @author Austin
 */
public class LogitechExtreme3D extends HID {

    public static final Axis STICK_X = new Axis(0, 0.05);
    public static final Axis STICK_Y = new Axis(1, 0.05, -1);
    public static final Axis STICK_ROT = new Axis(2, 0.05);
    public static final Button A = new Button(1);
    public static final Button B = new Button(2);
    public static final Button C = new Button(3);
    public static final Button D = new Button(4);
    public static final Button E = new Button(5);
    public static final Button F = new Button(6);
    public static final Button G = new Button(7);
    public static final Button H = new Button(8);
    public static final Button I = new Button(9);
    public static final Button J = new Button(10);
    public static final Button K = new Button(11);
    public static final Button L = new Button(12);
//we also need to account for the button on the top of the joystick that moves up and down, side to side, and diagonally. 
//I don't know how to do this--would we just take the x and y values and not account for the diagonal movement at all?
//Is accounting for this button at all even helpful in any way? ~Annalise

    /**
     *
     * @param port The port (1-4) that the controller is connected to in the
     * Driver Station.
     */
    public LogitechExtreme3D(int port) {
        super(port);
    }
}