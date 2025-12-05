package frc.robot.subsystems.turret;

/**
 * States for the turret subsystem describing its current behavior.
 */
public enum TurretState {
    /**
     * Default idle state; turret holds current position and awaits commands.
     */
    Default,
    /**
     * Move to the calibrated home position (zero/reference).
     */
    Home,
    /**
     * Actively track a target using vision or sensors.
     */
    Tracking,
    /**
     * Search for a target by sweeping or scanning the environment.
     * Goes to Default if Target is found.
     */
    Search
}