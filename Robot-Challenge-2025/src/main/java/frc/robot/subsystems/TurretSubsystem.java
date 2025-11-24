package frc.robot.subsystems;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LimelightHelpers;

public class TurretSubsystem extends SubsystemBase {

    private final double camOffsetX = 2;
    private final double camOffsetY = 3;
    private final double limelightAltitude = 10;
    //    Electronics
    private TalonFX turret_motor;
    private TalonFXConfiguration turret_config;

    private double Tx;
    private double Ty;
    private boolean Tv;
    private TurretState current_State;
    private PIDController controller;



    public TurretSubsystem() {
        current_State = TurretState.IDLE;

        turret_motor = new TalonFX(21);

        controller = new PIDController(0.05, 0, 0);
        controller.setTolerance(2);
        controller.enableContinuousInput(0, 360); // <<< add this

        turret_config = new TalonFXConfiguration();
        turret_config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        turret_config.Feedback.FeedbackRemoteSensorID = 31;
        turret_config.Feedback.RotorToSensorRatio = 25;

        // Soft limits ARE IN ROTATIONS, not degrees
        turret_config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 1;  // 1 rotation = 360°
        turret_config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

        turret_config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        turret_config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        // You MUST apply it
        turret_motor.getConfigurator().apply(turret_config);
    }



    public void periodic() {
        Tx = LimelightHelpers.getTX("limelight-vision");
        Ty = LimelightHelpers.getTY("limelight-vision");
        Tv = LimelightHelpers.getTV("limelight-vision");

        switch (current_State) {
            case TRACK_TARGET:
                stopMotor();
                break;
            case FIND_TARGET:
                ActionForFIND_TARGET();
            default:
                ActionForIDLE();
        }

    }

    public void ActionForIDLE() {
        stopMotor();
    }

    public void ActionForFIND_TARGET() {
        if (!targetPresent()) {
            startMotor(0.01);
        } else {
            stopMotor();
            controller.setSetpoint(wrapAngle(getTargetAngle()));
            startMotor(controller.calculate(getAngle()));

            if (controller.atSetpoint()) {
                current_State = TurretState.IDLE;
            }
        }
    }

    public void ActionForTRACK_TARGET() {
        if (!targetPresent()) {
            startMotor(0.01);
        } else {
            stopMotor();
            controller.setSetpoint(wrapAngle(getTargetAngle()));
            startMotor(controller.calculate(getAngle()));
        }
    }

    public void startMotor(double speed) {
        turret_motor.set(speed);
    }

    public void stopMotor() {
        turret_motor.stopMotor();
    }

    public double getAngle() {
        return turret_motor.getPosition().getValueAsDouble() * 360.0;
    }

    public double getDistance() {
        double limelightMountAngleDegrees = 0;
        double angleToGoalDegrees = limelightMountAngleDegrees + Ty;
        double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);

        return (36 - limelightAltitude) / Math.tan(angleToGoalRadians);
    }

    public double getTargetAngle() {
        double distance = getDistance();
        double txRad = Math.toRadians(Tx);

        double targetX = camOffsetX + distance * Math.sin(txRad);
        double targetY = camOffsetY + distance * Math.cos(txRad);

        return Math.toDegrees(Math.atan2(targetX, targetY));
    }


    public boolean targetPresent() {
        return Tv;
    }

    public Command SetToState(TurretState state) {
        return new InstantCommand(() -> current_State = state);
    }

    public double wrapAngle(double angleDeg) {
        angleDeg %= 360;
        return (angleDeg < 0) ? angleDeg + 360 : angleDeg;
    }


}

