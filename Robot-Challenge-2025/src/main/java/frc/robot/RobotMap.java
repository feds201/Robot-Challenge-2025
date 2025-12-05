package frc.robot;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Inch;

public class RobotMap {
    public static class TurretMap {
        public static final int TURRET_MOTOR_ID = 21;
        public static final int TURRET_ENCODER_ID = 31;
        public static final double MOTOR_TO_SENSOR = (double) 60; // 60:1
        public static final double SENSOR_TO_MECHANISM = 1 / 1.2; // 1:1.2
        public static final Distance TARGET_ALTITUDE = Distance.ofBaseUnits(36, Inch);
        public static final Distance LIMELIGHT_ALTITUDE = Distance.ofBaseUnits(18, Inch);
        public static final Angle LIMELIGHT_MOUNTING_ANGLE = Degree.of(15);
        public static final Distance CAMERA_OFFSET_X = Distance.ofBaseUnits(2, Inch);
        public static final Distance CAMERA_OFFSET_Y = Distance.ofBaseUnits(3, Inch);
    }
}
