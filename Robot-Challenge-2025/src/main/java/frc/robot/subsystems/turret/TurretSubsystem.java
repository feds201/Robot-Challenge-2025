package frc.robot.subsystems.turret;


import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.SlotConfig;

import static edu.wpi.first.units.Units.*;
import static frc.robot.RobotMap.TurretMap.*;

public class TurretSubsystem extends SubsystemBase {

    private final TalonFX turretMotor;

    private TurretState currentState;

    private Angle tX, tY;
    private Boolean tV;
    private Boolean searchDirectionRight;


    public TurretSubsystem() {

        currentState = TurretState.Default;

        turretMotor = new TalonFX(TURRET_MOTOR_ID);
        CANcoder turretEncoder = new CANcoder(TURRET_ENCODER_ID);

        TalonFXConfiguration motor_config = new TalonFXConfiguration();
        FeedbackConfigs motor_feedback_config = new FeedbackConfigs();
        SlotConfig motor_slot_config = new SlotConfig(3, 2, 2, 2, 2, 2, 2, GravityTypeValue.Elevator_Static);
        SoftwareLimitSwitchConfigs motor_software_limitSwitch_config = new SoftwareLimitSwitchConfigs();

        motor_feedback_config.SensorToMechanismRatio = SENSOR_TO_MECHANISM;
        motor_feedback_config.RotorToSensorRatio = MOTOR_TO_SENSOR;
        motor_feedback_config.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        motor_feedback_config.FeedbackRemoteSensorID = turretEncoder.getDeviceID();

        // Soft Limits (Protect Cables)
        motor_software_limitSwitch_config.ForwardSoftLimitEnable = true;
        motor_software_limitSwitch_config.ReverseSoftLimitEnable = true;
        // Limit to approx +/- 175 degrees to be safe
        motor_software_limitSwitch_config.ForwardSoftLimitThreshold = 0.48;
        motor_software_limitSwitch_config.ReverseSoftLimitThreshold = -0.48;


        motor_config.Feedback = motor_feedback_config;
        motor_config.SoftwareLimitSwitch = motor_software_limitSwitch_config;
        motor_config.Slot0 = Slot0Configs.from(motor_slot_config.getConfig(0));

        turretMotor.getConfigurator().apply(motor_config);

    }

    public Angle turretFacingAngle() {
        return turretMotor.getPosition().getValue();
    }

    public boolean targetAvailable() {
        return tV;
    }

    public Distance distanceCalculation() {
        // Units here depend on RobotMap. Ensure RobotMap uses Inches/Meters consistently.
        Distance opposite = TARGET_ALTITUDE.minus(LIMELIGHT_ALTITUDE);
        Angle theta = tY.plus(LIMELIGHT_MOUNTING_ANGLE);

        // Basic trig: d = h / tan(theta)
        double distance = opposite.in(Meter) / Math.tan(theta.in(Radians));
        return Distance.ofBaseUnits(distance, Meter);
    }

    public Angle targetAngleCalculation() {
        Distance distance = distanceCalculation();
        Distance opposite = distance.times(Math.tan(tX.in(Radians))).plus(CAMERA_OFFSET_X);
        Distance adjacent = distance.plus(CAMERA_OFFSET_Y);
        return Radians.of(Math.atan(opposite.in(Meter) / adjacent.in(Meter)));
    }

    public Command setState(TurretState state) {
        return new InstantCommand(() -> currentState = state, this);
    }

    public void ActDefault() {
        turretMotor.stopMotor();
        turretMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void ActHome() {
        turretMotor.setControl(new PositionVoltage(0).withPosition(0));
    }

    public void ActSearch() {
        double currentRotation = turretFacingAngle().in(Rotations);
        double searchSpeed = 0.26;

        if (currentRotation > 0.48) {
            searchDirectionRight = false;
        } else if (currentRotation < -0.48) {
            searchDirectionRight = true;
        }

        double finalVelocity = searchDirectionRight ? searchSpeed : -searchSpeed;

        turretMotor.setControl(new VelocityVoltage(0).withVelocity(RadiansPerSecond.of(finalVelocity)));
    }

    public void ActTracking() {
        Angle currentAngle = turretFacingAngle();
        Angle offsetAngle = targetAngleCalculation();
        Angle targetAngle = currentAngle.plus(offsetAngle);

        Angle clampedAngle = Rotations.of(Math.max(-0.48, Math.min(0.48, targetAngle.in(Rotations))));

        turretMotor.setControl(new PositionVoltage(0).withPosition(clampedAngle));
    }

    @Override
    public void periodic() {
        super.periodic();

        tX = Degrees.of(LimelightHelpers.getTX(""));
        tY = Degrees.of(LimelightHelpers.getTY(""));
        tV = LimelightHelpers.getTV("");


        switch (currentState) {
            case Home:
                ActHome();
                break;
            case Search:
                ActSearch();
                if (targetAvailable()) {
                    currentState = TurretState.Tracking;
                }
                break;
            case Tracking:
                ActTracking();
                if (!targetAvailable()) {
                    currentState = TurretState.Search;
                }
                break;
            default:
                ActDefault();


        }

    }
}

