package frc.robot;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.Encoder;

/**
 * A file for compatibility with Autodesk Synthesis robot simulation, which ensures PWM motors and Quadrature encoders, which link with Synthesis,
 * can be used as well as the physical encoders during simulation.
 */
class SimulationHardware {
    /**
     * --------------------------------------------------------------------------------------------------------------------------------------
     * The following code makes this act as a singleton class: https://www.geeksforgeeks.org/singleton-class-java/.
     * This means whenever you want to interact with SimulationHardware you can just use SimulationHardware.getInstance().<motor/encoder name>,
     * which will create the instance the first time, and use the same instance every next time. No initialisation is needed, so you don't 
     * need much extra code outside this file!
     */
    /**
     * Singleton instance.
     */
    private static SimulationHardware _instance;
    /**
     * Get the singleton instance, and set it up if it hasn't been created yet.
     */
    public static SimulationHardware getInstance() {
        if(SimulationHardware._instance == null) {
            SimulationHardware._instance = new SimulationHardware();
        }
        return SimulationHardware._instance;
    }
    /**
     * Constructor for the SimulationHardware instance, which creates motors using PWM and Quadrature encoders for use in Autodesk Synthesis simulation.
     * 
     * This is a private constructor so cannot be accessed directly. Use SimulationHardware.getInstance() to get the singleton instance instead.
     */
    private SimulationHardware() { // Private constructor
        this.leftDriveMotor = new VictorSP(0); // Port 0
        this.rightDriveMotor = new VictorSP(1); // Port 1
        this.leftDriveEncoder = new Encoder(0, 1); // Channels 0,1
        this.rightDriveEncoder = new Encoder(2, 3); // Channels 2,3
    }

    /**
     * --------------------------------------------------------------------------------------------------------------------------------------
     * The motors and encoders below used in the simulation can be accessed at SimulationHardware.getInstance().<motor/encoder name>, and will be
     * set up the first time they are called. You should surround any code using these with if(!RobotBase.isReal()) { } so it won't be run on the
     * actual robot.
     * 
     * If you want to add your own motors/encoders you should add them as fields below and also set them up in the constructor above. Use VictorSP for
     * motor controllers and Encoder for encoders.
     */

    public VictorSP leftDriveMotor;
    public VictorSP rightDriveMotor;
    public Encoder leftDriveEncoder;
    public Encoder rightDriveEncoder;
}