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
import frc.robot.utils.DrivetrainConstants;
import frc.robot.utils.RobotFramework;

import static frc.robot.utils.DrivetrainConstants.brake;

public class RobotContainer extends RobotFramework {
//  private final Arm arm = new Arm();

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
        drivetrain.setDefaultCommand(
                TeleOpBuilder.buildTankDrive(DrivetrainConstants.getMaxSpeed(), DrivetrainConstants.getMaxAngularSpeed())
        );
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(turretSubsystem.setState(TurretState.Search));
        joystick.y().whileTrue(turretSubsystem.setState(TurretState.Tracking));



        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

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
