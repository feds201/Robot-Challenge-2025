// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.turret.TurretState;
import frc.robot.subsystems.turret.TurretSubsystem;

public class RobotContainer {
    private CommandXboxController controller = new CommandXboxController(0);
    private TurretSubsystem turretSubsystem = new TurretSubsystem();
  public RobotContainer() {
    configureBindings();
  }

    private void configureBindings() {
        controller.x().onTrue(turretSubsystem.SetToState(TurretState.TRACK_TARGET));
    }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
