package frc.robot.subsystems.turret;


import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

public class Turret2Subsystem extends SubsystemBase {
    TalonFX motor1 = new TalonFX(61);
    CANcoder CANCODER = new CANcoder(62);
    public PIDController controller = new PIDController(3.5, 0, .25);
    TalonFXConfiguration config2 = new TalonFXConfiguration();


    public Turret2Subsystem() {
        config2.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        config2.Feedback.FeedbackRemoteSensorID = CANCODER.getDeviceID();
        config2.Feedback.RotorToSensorRatio = 60;
        config2.Feedback.SensorToMechanismRatio =0.833;
    }
    @Override

    public void periodic() {
        SmartDashboard.putNumber("Motor's pos", motor1.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("CANcoder's pos",  CANCODER.getAbsolutePosition().getValue().in(Degrees));

    }


    public void runTurret(double speed) {
        motor1.set(speed);
    }



    public void stopTurret() {
        motor1.stopMotor();
    }

    public double getPosition() {
        return motor1.getPosition().getValue().in(Degrees); // after change
//        return CANCODER1.getPosition().getValue().in(Degrees); // before change
    }


}

