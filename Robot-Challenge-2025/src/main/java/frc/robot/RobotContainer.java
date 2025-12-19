package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.turret.Turret2Subsystem;
import frc.robot.subsystems.turret.TurretState;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.utils.DrivetrainConstants;
import frc.robot.utils.RobotFramework;

import static frc.robot.utils.DrivetrainConstants.brake;

public class RobotContainer extends RobotFramework {
//  private final Arm arm = new Arm();



    //   private TurretSubsystem turretSubsystem = new TurretSubsystem();
    // public Turret2Subsystem turret_subsystem =  new Turret2Subsystem();
    public TurretSubsystem turretSubsystem = new TurretSubsystem();
    
    public double test;
  public RobotContainer() {
    configureBindings();
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


    }



    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
