package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.SlotConfig;
import static edu.wpi.first.units.Units.*;
import static frc.robot.RobotMap.TurretMap.*;

public class TurretSubsystem extends SubsystemBase {

    /**
     * The TalonFX motor controller for the turret.
     */
    private final TalonFX turretMotor;
    private final CANcoder turretEncoder;

    // 1. Handles for the CTRE Simulation "State"
    // private final TalonFXSimState motorSimState;
    // private final CANcoderSimState encoderSimState;

    // 2. The WPILib Physics Engine
    // private final DCMotorSim turretSim;

    private TurretState currentState;

    private Angle tX, tY;
    private Boolean tV;
    private Boolean searchDirectionRight = true;
    private PIDController laserTargetter = new PIDController(0.2, 0, 0);
    




    public TurretSubsystem() {
        
        currentState = TurretState.Default;
        laserTargetter.setTolerance(2);
        turretMotor = new TalonFX(TURRET_MOTOR_ID);
        turretEncoder = new CANcoder(TURRET_ENCODER_ID);

        // motorSimState = turretMotor.getSimState();
        // encoderSimState = turretEncoder.getSimState();

        tV = true;
        tX = Degree.of(0);
        tY = Degree.of(0);

        CANcoderConfiguration encoder_config = new CANcoderConfiguration();
        encoder_config.MagnetSensor.MagnetOffset = 0.5;
        encoder_config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        encoder_config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;

        turretEncoder.getConfigurator().apply(encoder_config);

        TalonFXConfiguration motor_config = new TalonFXConfiguration();
        FeedbackConfigs motor_feedback_config = new FeedbackConfigs();
        SlotConfig motor_slot_config = new SlotConfig(0.6, 0, 0, 0, 0, 0, 0, GravityTypeValue.Elevator_Static);
        SoftwareLimitSwitchConfigs motor_software_limitSwitch_config = new SoftwareLimitSwitchConfigs();

        motor_feedback_config.SensorToMechanismRatio = SENSOR_TO_MECHANISM;
        motor_feedback_config.RotorToSensorRatio = 0;
        motor_feedback_config.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        motor_feedback_config.FeedbackRemoteSensorID = turretEncoder.getDeviceID();

        // Soft Limits (Protect Cables)
        motor_software_limitSwitch_config.ForwardSoftLimitEnable = false;
        motor_software_limitSwitch_config.ReverseSoftLimitEnable = false; 
        // Limit to approx +/- 175 degrees to be safe
        motor_software_limitSwitch_config.ForwardSoftLimitThreshold = 1.4;
        motor_software_limitSwitch_config.ReverseSoftLimitThreshold = 0.4;

        motor_config.Feedback = motor_feedback_config;
        motor_config.SoftwareLimitSwitch = motor_software_limitSwitch_config;
        motor_config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motor_config.Slot0 = Slot0Configs.from(motor_slot_config.getConfig(0));

        turretMotor.getConfigurator().apply(motor_config);

        turretMotor.setPosition(turretEncoder.getAbsolutePosition().getValue());

        // var turretPlant = LinearSystemId.createDCMotorSystem(
        //     DCMotor.getFalcon500(1), 
        //     0.005, // Moment of Inertia
        //     50.0   // Gear Ratio (GEAR_RATIO from your Map)
        // );

        // turretSim = new DCMotorSim(turretPlant, DCMotor.getFalcon500(1));

    }

    /**
     * Returns the current angle the turret is facing.
     *
     * @return The current angle of the turret in rotations.
     */
    public Angle turretFacingAngle() {
        return turretMotor.getPosition().getValue();
    }

    /**
     * Checks if a target is currently available according to Limelight.
     *
     * @return True if a target is available, false otherwise.
     */
    public boolean targetAvailable() {
        return tV;
    }

    /**
     * Calculates the horizontal distance to the target based on Limelight data.
     *
     * @return The calculated distance to the target.
     */
    public Distance distanceCalculation() {
        Distance opposite = TARGET_ALTITUDE.minus(LIMELIGHT_ALTITUDE);
        Angle theta = tY.plus(LIMELIGHT_MOUNTING_ANGLE);

        // Basic trig: d = h / tan(theta)
        double distance = opposite.in(Meter) / Math.tan(theta.in(Radians));
        return Distance.ofBaseUnits(distance, Meter);
    }



    /**
     * Calculates the angle the turret needs to turn to face the target.
     *
     * @return The calculated angle offset to the target.
     */
    public Angle targetAngleCalculation() {
        Distance distance = distanceCalculation();
        Distance opposite = distance.times(Math.tan(tX.in(Radians))).plus(CAMERA_OFFSET_X);
        Distance adjacent = distance.plus(CAMERA_OFFSET_Y);
        return Radians.of(Math.atan(opposite.in(Meter) / adjacent.in(Meter)));
    }

    /**
     * Sets the current state of the turret.
     *
     * @param state The new state for the turret.
     * @return An InstantCommand that sets the turret's state.
     */
    public Command setState(TurretState state) {
        return new InstantCommand(() -> currentState = state, this);
    }

    /**
     * Implements the default behavior for the turret, which is to stop and brake.
     */
    public void ActDefault() {
        turretMotor.stopMotor();
        // turretMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    /**
     * Commands the turret to return to its home position (0 rotations).
     */
    public void ActHome() {
        // turretMotor.setControl(new PositionVoltage(turretFacingAngle()).withPosition(0));
    }

    /**
     * Implements the search behavior for the turret, rotating back and forth within its limits.
     */
    public void ActSearch() {
        double currentRotation = turretFacingAngle().in(Rotations);
        double searchSpeed = 0.05;

        if (currentRotation > 0.4) {
            searchDirectionRight = false;
        } else if (currentRotation < 1.3) {
            searchDirectionRight = true;
        }

        double finalVelocity = searchDirectionRight ? searchSpeed : -searchSpeed;
        SmartDashboard.putNumber("PID Val", finalVelocity);
        turretMotor.set(finalVelocity);
    }

    /**
     * Implements the tracking behavior for the turret, aiming at the detected target.
     */
public void ActTracking() {
    Angle currentAngle = turretFacingAngle();
    Angle offsetAngle = targetAngleCalculation();
    Angle targetAngle = currentAngle.plus(offsetAngle);
    
    // Calculate raw PID output
    double rawOutput = laserTargetter.calculate(currentAngle.in(Degree), targetAngle.in(Degree));
    SmartDashboard.putNumber("Turret RAW", rawOutput);
    
    // 1. Scale it down if needed (optional, depends on your PID tuning)
    double scaledOutput = rawOutput * 0.1; 
    SmartDashboard.putNumber("Turret Scaled", scaledOutput);


    // 2. Clamp the result strictly between -1 and 1
    double clampedOutput = MathUtil.clamp(scaledOutput, -1.0, 1.0);

    SmartDashboard.putNumber("Turret PID", clampedOutput);

    if (tV) {
        turretMotor.set(clampedOutput);
    }
}


    /**
     * This method is called periodically by the scheduler. It updates Limelight data and executes the current turret state's action.
     */
    @Override

    public void periodic() {
        SmartDashboard.putNumber("Turret/turret position", turretMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Turret/encoder value", turretEncoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("Limelight/TX", tX.in(Degree));
        SmartDashboard.putNumber("Limelight/TY", tY.in(Degree));
        SmartDashboard.putBoolean("Limelight/TV", tV);
        SmartDashboard.putString("Turret State", currentState.toString());
        // tX = Degrees.of(LimelightHelpers.getTX("limelight-vision"));
        // tY = Degrees.of(LimelightHelpers.getTY("limelight-vision"));
        tV = LimelightHelpers.getTV("limelight-vision");
        ActTracking();
        // switch (currentState) {
        //     case Home:
        //         ActHome();
        //         break;
        //     case Search:
        //         ActSearch();
        //         if (targetAvailable()) {
        //             currentState = TurretState.Tracking;
        //             turretMotor.stopMotor();

        //         }
        //         break;
        //     case Tracking:
                
        //         ActTracking();
        //         if (!targetAvailable()) {
        //             currentState = TurretState.Search;
        //         }
        //         break;
        //     default:
        //         ActDefault();

        // }


    }

    @Override
    public void simulationPeriodic() {
        tX = Degree.of(Math.random()*10);
        tY = Degree.of(Math.random()*10);
        // motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        // encoderSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        // double motorOutputVoltage = motorSimState.getMotorVoltage();

        // turretSim.setInputVoltage(motorOutputVoltage);

        // turretSim.update(0.020);

        // double mechanismPos = turretSim.getAngularPositionRotations();
        // double mechanismVel = turretSim.getAngularVelocityRPM() / 60.0; // convert to Rotations/Sec

        // // Use your GEAR_RATIO to tell the Talon what the motor's shaft is doing
        // motorSimState.setRawRotorPosition(mechanismPos * SENSOR_TO_MECHANISM);
        // motorSimState.setRotorVelocity(mechanismVel * SENSOR_TO_MECHANISM);

        // // Update the CANcoder (which is usually on the mechanism itself)
        // encoderSimState.setRawPosition(mechanismPos);
        // encoderSimState.setVelocity(mechanismVel);
    }
}
