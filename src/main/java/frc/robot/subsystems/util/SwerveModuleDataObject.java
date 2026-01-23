package frc.robot.subsystems.util;

import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModuleDataObject {
    
    public record SwerveModuleData(SwerveModuleState state) {
        
    }
}
