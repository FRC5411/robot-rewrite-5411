package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder.Feeder;


public class DriverIntakeFeedback extends Command{
    
    private Feeder feeder;
    private Joystick driver;
    private Joystick operator;

    public DriverIntakeFeedback (Feeder feeder, Joystick driver, Joystick operator){

        this.feeder = feeder;
        this.driver = driver;
        this.operator = operator;
    }

    @Override
    public void execute() {
        if (feeder.getIndexerSensor() || feeder.getIntakeSensor()) {
            driver.setRumble(RumbleType.kBothRumble, 1);
            operator.setRumble(RumbleType.kBothRumble, 1);
        } else {
            driver.setRumble(RumbleType.kBothRumble, 0);
            operator.setRumble(RumbleType.kBothRumble, 0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        driver.setRumble(RumbleType.kBothRumble, 0);
        operator.setRumble(RumbleType.kBothRumble, 0);
    }
}
