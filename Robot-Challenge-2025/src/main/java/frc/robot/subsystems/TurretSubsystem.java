// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LimelightHelpers;

public class TurretSubsystem extends SubsystemBase {
    private TalonFX turret_motor = new TalonFX(21);
    private CANcoder turret_encoder = new CANcoder(31);

    private double Tx;
    private double Ty;
    private boolean Tv;

    private TurretState cTurretState = TurretState.IDLE;
    private PIDController controller = new PIDController(0, 0, 0);

    // Limelight physical offsets relative to turret center
    private final double camOffsetX = 3.0; // inches left (+) or right (-)
    private final double camOffsetY = 2.0; // inches forward (+) or backward (-)

    double limelightMountAngleDegrees = 41;
    double limelightLensHeightInches = 25;
    double goalHeightInches = 15; // replace with actual height

    public enum TurretState {
        LOCKING_TARGET, SPIN, FIND_TARGET, IDLE;
    }

    public TurretSubsystem() {
    }

    @Override
    public void periodic() {

        Tv = LimelightHelpers.getTV("limelight");
        Tx = LimelightHelpers.getTX("limelight");
        Ty = LimelightHelpers.getTY("limelight");

        if (cTurretState == TurretState.SPIN) {

            startMotor(0.1);

        } else if (cTurretState == TurretState.FIND_TARGET) {

            if (!targetAvailable()) {

                startMotor(0.01); // slow scan

            } else {

                stopMotor();

                controller.setSetpoint(targetAngle());
                double output = controller.calculate(getAngle());
                startMotor(output);

                if (controller.atSetpoint()) {
                    GoToState(TurretState.IDLE);
                }
            }
        }
    }

    public void startMotor(double speed) {
        turret_motor.set(speed);
    }

    public void stopMotor() {
        turret_motor.stopMotor();
    }

    public double getAngle() {
        // assumes CANcoder is configured to return degrees
        return turret_encoder.getPosition().getValueAsDouble();
    }

    private double getDistance() {

        double angleToGoalDegrees = limelightMountAngleDegrees + Ty;
        double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);

        return (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
    }

    /**
     * Computes turret aiming angle based on:
     * - Limelight Tx
     * - distance to target
     * - camera offset relative to turret center
     */
    private double targetAngle() {

        double distance = getDistance();

        double txRad = Math.toRadians(Tx);

        // Target position relative to turret center
        double targetX = camOffsetX + distance * Math.sin(txRad);
        double targetY = camOffsetY + distance * Math.cos(txRad);

        // Return angle (radians) turret should point toward
        return Math.atan2(targetX, targetY);
    }

    public boolean targetAvailable() {
        return Tv;
    }

    public void GoToState(TurretState targetState) {
        cTurretState = targetState;
    }
}
