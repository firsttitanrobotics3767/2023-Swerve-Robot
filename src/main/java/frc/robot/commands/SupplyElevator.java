package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class SupplyElevator extends CommandBase{
    private final Elevator elevator;
    private final Supplier<Double> suppliedVolts;

    public SupplyElevator (Supplier<Double> suppliedVolts, Elevator elevator) {
        this.suppliedVolts = suppliedVolts;
        this.elevator = elevator;

        addRequirements(elevator);
    }

    @Override 
    public void execute() {
        elevator.setVolts(suppliedVolts.get());
    }

    @Override
    public void end(boolean isInterrupted) {
        elevator.setVolts(0);
    }
}
