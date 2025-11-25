package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class TurretTrigger extends Trigger {

    private final TurretSubsystem turret;
    private final TurretState watchedState;

    public TurretTrigger(TurretSubsystem turret, TurretState watchedState) {
        // Trigger activates when this supplier returns true
        super(() -> turret.getState() == watchedState);

        this.turret = turret;
        this.watchedState = watchedState;
    }

    public boolean get() {
        return turret.getState() == watchedState;
    }
}
