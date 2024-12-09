package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Chassis;

public class Drive extends Command {
    private Chassis chassis;
    private Joystick joystick;

    public Drive(Chassis chassis, Joystick joystick) {
        this.chassis = chassis;
        this.joystick = joystick;

        addRequirements(chassis);
    }
    
    @Override
    public void execute() {
        double inputDrive = joystick.getY();
        double inputSteer = joystick.getTwist();
        ChassisSpeeds speeds =  new ChassisSpeeds(-inputDrive * 4, 0, inputSteer * 2 * 2 * Math.PI);
        chassis.setVelocity(speeds);
    }
}
