package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.armState;
import frc.robot.subsystems.turret.TurretState;
import frc.robot.subsystems.turret.TurretSubsystem;

public class RobotContainer {
  private final Arm arm = new Arm();
  private final CommandXboxController controller = new CommandXboxController(0);
   private TurretSubsystem turretSubsystem = new TurretSubsystem();
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    controller.a().onTrue(arm.commandState(armState.TARGETING));
    controller.b().onTrue(arm.commandState(armState.STOW));
    controller.y().onTrue(arm.commandState(armState.ZERO));
    controller.x().onTrue(turretSubsystem.setState(TurretState.Default));

    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
