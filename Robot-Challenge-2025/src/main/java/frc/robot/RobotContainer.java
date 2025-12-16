package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.armState;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.generated.TunerConstants;
import frc.robot.subsystems.turret.TurretState;
import frc.robot.subsystems.turret.TurretSubsystem;

public class RobotContainer {
  private final Arm arm = new Arm();
  private final CommandXboxController controller = new CommandXboxController(0);
  private CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Telemetry telemetry = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));
   private TurretSubsystem turretSubsystem = new TurretSubsystem();
  public RobotContainer() {
    configureBindings();
    drivetrain.registerTelemetry(telemetry::telemeterize);
  }

  private void configureBindings() {
    controller.a().onTrue(arm.commandState(armState.TARGETING));
    controller.b().onTrue(arm.commandState(armState.STOW));
    controller.y().onTrue(arm.commandState(armState.ZERO));
    controller.x().onTrue(turretSubsystem.setState(TurretState.Default));
    drivetrain.setDefaultCommand(new TeleopSwerve(drivetrain, controller));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
