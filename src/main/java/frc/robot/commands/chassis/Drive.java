package frc.robot.commands.chassis;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.chassis.Chassis;

public class Drive extends Command {
    private Chassis chassis;
    private CommandXboxController controller;

    public Drive(Chassis chassis, CommandXboxController controller) {
        this.chassis = chassis;
        this.controller = controller;

        addRequirements(chassis);
    }

    @Override
    public void execute() {
        chassis.setVelocities(new ChassisSpeeds(controller.getLeftX() * 2, controller.getLeftY() * 2, 0));
    }
}
