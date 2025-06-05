package frc.robot.controllers;

public enum LoggingVerbosity {
    /**Only show errors */
    MINIMUM(0),
    /**Show warnings but without stack traces*/
    LOW(1),
    /**Show warnings but with stack traces*/
    HIGH(2),
    /**Show absolutely everything, may spam console pretty heavily */
    EVERYTHING(3);
    
    private final int val;
    
    LoggingVerbosity(int _val) {
        val = _val;
    }
    
    public int asInt() {
        return val;
    }
}
