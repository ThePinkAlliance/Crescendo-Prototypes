package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class Joy extends Command {
    
    //private final Shooter m_subsystem;

    Joystick controller = new Joystick(0);

    @Override
    public void execute() {

        double LS_X = controller.getRawAxis(0);
        double LS_Y = controller.getRawAxis(1);

        //m_subsystem.ramp(-RS_Y, LS_Y);
    }
}
