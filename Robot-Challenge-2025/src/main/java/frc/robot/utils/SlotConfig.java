package frc.robot.utils;

import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;

public class SlotConfig {
    private static double kP = 0.0;
    private static double kI = 0.0;
    private static double kD = 0.0;
    private static double kS = 0.0;
    private static double kV = 0.0;
    private static double kA = 0.0;
    private static double kG = 0.0;
    private static GravityTypeValue gravityType = GravityTypeValue.Elevator_Static;

    // Constructors
    public SlotConfig(double kP, double kI, double kD, double kS, double kV, double kA, double kG, GravityTypeValue gravityType) {
        SlotConfig.kP = kP;
        SlotConfig.kI = kI;
        SlotConfig.kD = kD;
        SlotConfig.kS = kS;
        SlotConfig.kV = kV;
        SlotConfig.kA = kA;
        SlotConfig.kG = kG;
        SlotConfig.gravityType = gravityType;
    }

    public SlotConfig(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
        SlotConfig.kP = kP;
        SlotConfig.kI = kI;
        SlotConfig.kD = kD;
        SlotConfig.kS = kS;
        SlotConfig.kV = kV;
        SlotConfig.kA = kA;
        SlotConfig.kG = kG;
    }

    public SlotConfig(double kP, double kI, double kD, double kS, double kV) {
        SlotConfig.kP = kP;
        SlotConfig.kI = kI;
        SlotConfig.kD = kD;
        SlotConfig.kS = kS;
        SlotConfig.kV = kV;
    }

    public SlotConfig(double kP, double kI, double kD, double kS) {
        SlotConfig.kP = kP;
        SlotConfig.kI = kI;
        SlotConfig.kD = kD;
        SlotConfig.kS = kS;
    }

    public SlotConfig(double kP, double kI, double kD) {
        SlotConfig.kP = kP;
        SlotConfig.kI = kI;
        SlotConfig.kD = kD;
    }

    public SlotConfig(double kP, double kI) {
        SlotConfig.kP = kP;
        SlotConfig.kI = kI;
    }

    public SlotConfig(double kP) {
        SlotConfig.kP = kP;
    }

    public SlotConfig() {
    }

    // Get a configured SlotConfigs object
    public SlotConfigs getConfig(int slotNumber) {
        SlotConfigs slotConfigs = new SlotConfigs();
        slotConfigs.kP = kP;
        slotConfigs.kI = kI;
        slotConfigs.kD = kD;
        slotConfigs.kS = kS;
        slotConfigs.kV = kV;
        slotConfigs.kA = kA;
        slotConfigs.kG = kG;
        slotConfigs.GravityType = gravityType;
        slotConfigs.SlotNumber = slotNumber;
        return slotConfigs;
    }
}
