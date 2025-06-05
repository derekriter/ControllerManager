package frc.robot.controllers;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * A class that handles logging for all custom controllers
 */
public abstract class ControllerLogger {
    
    public static final LoggingVerbosity DEFAULT_VERBOSITY = LoggingVerbosity.HIGH;
    
    private static final String PREFIX = "ControllerManager - ";
    private static LoggingVerbosity verbosity = DEFAULT_VERBOSITY;
    
    public static void setVerbosity(LoggingVerbosity newVerbosity) {
        System.out.printf("%sChanging verbosity from %s to %s\n", PREFIX, verbosity.toString(), newVerbosity.toString()); //TODO try to find a more performant way of doing this
        
        verbosity = newVerbosity;
    }
    public static LoggingVerbosity getVerbosity() {
        return verbosity;
    }
    
    /*
     * Minimum
     */
    static void errorOutOfRange(String valName, int val, String range) throws IllegalArgumentException {
        throw new IllegalArgumentException(String.format("%sArgument '%s' with value %d out of range %s", PREFIX, valName, val, range));
    }
    static void errorNullArgument(String valName) throws IllegalArgumentException {
        throw new IllegalArgumentException(String.format("%sArgument '%s' cannot be null", PREFIX, valName));
    }
    static void errorInvalidRumble(int controller, int rumble) throws IllegalArgumentException {
        throw new IllegalArgumentException(String.format("%sNo rumble with id %d is currently active on controller %d", PREFIX, rumble, controller));
    }
    
    /*
     * LOW & HIGH
     */
    static void warningButtonUnderMin(int button) {
        if(verbosity.asInt() < LoggingVerbosity.LOW.asInt()) return;
        
        boolean showTrace = verbosity.asInt() >= LoggingVerbosity.HIGH.asInt();
        DriverStation.reportWarning(String.format("%sInvalid button ID %d, button IDs should be >= 1", PREFIX, button), showTrace);
    }
    static void warningButtonOverMax(int port, int button, int maxID) {
        if(verbosity.asInt() < LoggingVerbosity.LOW.asInt()) return;
        
        boolean showTrace = verbosity.asInt() >= LoggingVerbosity.HIGH.asInt();
        DriverStation.reportWarning(String.format("%sInvalid button ID %d, controller %d has a maximum button ID of %d", PREFIX, button, port, maxID), showTrace);
    }
    static void warningAxisUnderMin(int axis) {
        if(verbosity.asInt() < LoggingVerbosity.LOW.asInt()) return;
        
        boolean showTrace = verbosity.asInt() >= LoggingVerbosity.HIGH.asInt();
        DriverStation.reportWarning(String.format("%sInvalid axis ID %ds, axis IDs should be >= 0", PREFIX, axis), showTrace);
    }
    static void warningAxisOverMax(int port, int axis, int maxID) {
        if(verbosity.asInt() < LoggingVerbosity.LOW.asInt()) return;
        
        boolean showTrace = verbosity.asInt() >= LoggingVerbosity.HIGH.asInt();
        DriverStation.reportWarning(String.format("%sInvalid axis ID %d, controller %d has a maximum axis ID of %d", PREFIX, axis, port, maxID), showTrace);
    }
    static void warningReachedRumbleLimit(int maxRumbles) {
        if(verbosity.asInt() < LoggingVerbosity.LOW.asInt()) return;
        
        boolean showTrace = verbosity.asInt() >= LoggingVerbosity.HIGH.asInt();
        DriverStation.reportWarning(String.format("%sFailed to schedule rumble. The rumble limit of %d has been reached.", PREFIX, maxRumbles), showTrace);
    }
    
    /*
     * EVERYTHING
     */
    static void warningClamp(String valName, int val, String range) {
        if(verbosity.asInt() < LoggingVerbosity.EVERYTHING.asInt()) return;
        
        DriverStation.reportWarning(String.format("%sClamped argument '%s' with value %d to range %s", PREFIX, valName, val, range), true);
    }
    static void warningClamp(String valName, double val, String range) {
        if(verbosity.asInt() < LoggingVerbosity.EVERYTHING.asInt()) return;
        
        DriverStation.reportWarning(String.format("%sClamped argument '%s' with value %f to range %s", PREFIX, valName, val, range), true);
    }
}
