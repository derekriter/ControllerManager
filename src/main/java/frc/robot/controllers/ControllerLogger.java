package frc.robot.controllers;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * For internal use only, is package private
 */
abstract class ControllerLogger {
    
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
    public static void errorControllerAlreadyExists(int controller) throws IllegalArgumentException {
        throw new IllegalArgumentException(String.format("%sCannot register multiple controllers with id %d", PREFIX, controller));
    }
    public static void errorOutOfRange(String valName, int val, String range) throws IllegalArgumentException {
        throw new IllegalArgumentException(String.format("%sArgument '%s' with value %d out of range %s", PREFIX, valName, val, range));
    }
    public static void errorNullArgument(String valName) throws IllegalArgumentException {
        throw new IllegalArgumentException(String.format("%sArgument '%s' cannot be null", PREFIX, valName));
    }
    public static void errorHasNotInited() throws IllegalStateException {
        throw new IllegalStateException(String.format("%sControllerManager has not been initialized", PREFIX));
    }
    public static void errorInvalidController(int controller) throws IllegalArgumentException {
        throw new IllegalArgumentException(String.format("%sNo controller with id %d has been registered", PREFIX, controller));
    }
    public static void errorInvalidRumble(int controller, int rumble) throws IllegalArgumentException {
        throw new IllegalArgumentException(String.format("%sNo rumble with id %d is currently active on controller %d", PREFIX, rumble, controller));
    }
    
    /*
     * LOW & HIGH
     */
    public static void warningInvalidController(int controller) {
        if(verbosity.asInt() < LoggingVerbosity.LOW.asInt()) return;
        
        boolean showTrace = verbosity.asInt() >= LoggingVerbosity.HIGH.asInt();
        DriverStation.reportWarning(String.format("%sNo controller with the id %d has been registered", PREFIX, controller), showTrace);
    }
    public static void warningButtonUnderMin(int button) {
        if(verbosity.asInt() < LoggingVerbosity.LOW.asInt()) return;
        
        boolean showTrace = verbosity.asInt() >= LoggingVerbosity.HIGH.asInt();
        DriverStation.reportWarning(String.format("%sInvalid button ID %d, button IDs should be >= 1", PREFIX, button), showTrace);
    }
    public static void warningButtonOverMax(int controller, int button, int maxID) {
        if(verbosity.asInt() < LoggingVerbosity.LOW.asInt()) return;
        
        boolean showTrace = verbosity.asInt() >= LoggingVerbosity.HIGH.asInt();
        DriverStation.reportWarning(String.format("%sInvalid button ID %d, controller %d has a maximum button ID of %d", PREFIX, button, controller, maxID), showTrace);
    }
    public static void warningAxisUnderMin(int axis) {
        if(verbosity.asInt() < LoggingVerbosity.LOW.asInt()) return;
        
        boolean showTrace = verbosity.asInt() >= LoggingVerbosity.HIGH.asInt();
        DriverStation.reportWarning(String.format("%sInvalid axis ID %ds, axis IDs should be >= 0", PREFIX, axis), showTrace);
    }
    public static void warningAxisOverMax(int controller, int axis, int maxID) {
        if(verbosity.asInt() < LoggingVerbosity.LOW.asInt()) return;
        
        boolean showTrace = verbosity.asInt() >= LoggingVerbosity.HIGH.asInt();
        DriverStation.reportWarning(String.format("%sInvalid axis ID %d, controller %d has a maximum axis ID of %d", PREFIX, axis, controller, maxID), showTrace);
    }
    public static void warningReachedRumbleLimit(int maxRumbles) {
        if(verbosity.asInt() < LoggingVerbosity.LOW.asInt()) return;
        
        boolean showTrace = verbosity.asInt() >= LoggingVerbosity.HIGH.asInt();
        DriverStation.reportWarning(String.format("%sFailed to schedule rumble. The rumble limit of %d has been reached.", PREFIX, maxRumbles), showTrace);
    }
    
    /*
     * EVERYTHING
     */
    public static void warningClamp(String valName, int val, String range) {
        if(verbosity.asInt() < LoggingVerbosity.EVERYTHING.asInt()) return;
        
        DriverStation.reportWarning(String.format("%sClamped argument '%s' with value %d to range %s", PREFIX, valName, val, range), true);
    }
    public static void warningClamp(String valName, double val, String range) {
        if(verbosity.asInt() < LoggingVerbosity.EVERYTHING.asInt()) return;
        
        DriverStation.reportWarning(String.format("%sClamped argument '%s' with value %f to range %s", PREFIX, valName, val, range), true);
    }
}
