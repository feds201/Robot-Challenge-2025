package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.Turret2Subsystem;


public class TurnCommand extends Command {
    private final Turret2Subsystem turret2Subsystem;

    private double angle;
    public TurnCommand(Turret2Subsystem turret2Subsystem, double targetAngle) {
        this.turret2Subsystem = turret2Subsystem;
        // each subsystem used by the command must be passed into the
        this.angle = targetAngle;
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.turret2Subsystem);
    }

    @Override
    public void initialize() {
        turret2Subsystem.controller.setSetpoint(angle);
        turret2Subsystem.controller.setTolerance(2);
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("Aligning Angle", turret2Subsystem.getPosition());
        SmartDashboard.putNumber("Target Angle", turret2Subsystem.controller.getSetpoint());

        double hi = turret2Subsystem.controller.calculate(turret2Subsystem.getPosition());
        turret2Subsystem.runTurret(Math.min(Math.max(hi,-0.15),0.15));
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return turret2Subsystem.controller.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        turret2Subsystem.stopTurret();
    }
}
