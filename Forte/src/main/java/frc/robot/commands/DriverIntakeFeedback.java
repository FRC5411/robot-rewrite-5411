package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.IndexerIntake.IndexerIntake;


public class DriverIntakeFeedback extends Command{
    
    private IndexerIntake intake;
    private CommandXboxController driver;
    private CommandXboxController operator;

    public DriverIntakeFeedback (IndexerIntake intake, CommandXboxController driver, CommandXboxController operator){

        this.intake = intake;
        this.driver = driver;
        this.operator = operator;
    }

    @Override
    public void execute() {
        if (intake.IndexerSensorHasNote() || intake.IntakeSensorHasNote()) {
            driver.getHID().setRumble(RumbleType.kBothRumble, 1);
            operator.getHID().setRumble(RumbleType.kBothRumble, 1);
        } else {
            driver.getHID().setRumble(RumbleType.kBothRumble, 0);
            operator.getHID().setRumble(RumbleType.kBothRumble, 0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        driver.getHID().setRumble(RumbleType.kBothRumble, 0);
        operator.getHID().setRumble(RumbleType.kBothRumble, 0);
    }
}
