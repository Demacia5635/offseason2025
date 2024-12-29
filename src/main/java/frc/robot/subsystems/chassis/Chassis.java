package frc.robot.subsystems.chassis;

import com.ctre.phoenix6.hardware.Pigeon2;

public class Chassis {
    private SwerveModule[] modules;
    private Pigeon2 gyro;

    public Chassis() {
        modules = new SwerveModule[] {
            new SwerveModule(ChassisConstants.ModuleConstants.FRONT_LEFT_STEER, ChassisConstants.ModuleConstants.FRONT_LEFT_DRIVE),
            new SwerveModule(ChassisConstants.ModuleConstants.FRONT_RIGHT_STEER, ChassisConstants.ModuleConstants.FRONT_RIGHT_DRIVE),
            new SwerveModule(ChassisConstants.ModuleConstants.BACK_LEFT_STEER, ChassisConstants.ModuleConstants.BACK_LEFT_DRIVE),
            new SwerveModule(ChassisConstants.ModuleConstants.BACK_RIGHT_STEER, ChassisConstants.ModuleConstants.BACK_RIGHT_DRIVE),
        };
        gyro = new Pigeon2(ChassisConstants.GYRO_ID);
    }
}
