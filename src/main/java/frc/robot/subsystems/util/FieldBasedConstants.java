package frc.robot.subsystems.util;

import edu.wpi.first.wpilibj.DriverStation;

public class FieldBasedConstants {
        public static boolean isBlueAlliance() {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue;
    }
}
