package frc.robot.utility;

import java.lang.Object;
import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Util {
    public static NetworkTable table = NetworkTableInstance.getDefault().getTable("Debug");

    
    public static Object get(String DataName) {
        Object obj = table.getValue(DataName).getValue();
        if (obj == null) {
            set(DataName, 0.0);
            return get(DataName);
        }
        return obj;
    }

    public static Object get(String DataName, Object defaultValue) {
        Object obj = table.getValue(DataName).getValue();
        if (obj == null) {
            set(DataName, defaultValue);
            return defaultValue;
        }
        return obj;
    }

    public static void set(String DataName, Object Data) {
        table.getEntry(DataName).setValue(Data);
    }

    public static InstantCommand Do(Runnable action, Subsystem... systems) {
        return new InstantCommand(action, systems);
    }

    public static Command Do(Runnable action, Runnable end, Subsystem... systems) {
        return Commands.runEnd(action, end, systems);
    }

    public static InstantCommand blank = new InstantCommand();

    public static Command DoCheck(Command cmd, BooleanSupplier condition, Subsystem... systems){
        return new ConditionalCommand(cmd, blank, condition);
    }

    public static Command DoUntil(Runnable action, Runnable end, BooleanSupplier Condition, Subsystem... systems) {
        return Commands.runEnd(action, end, systems).until(Condition);
    }

    public static ConditionalCommand Do(Command onTrue, Command onFalse, BooleanSupplier condition){
        return new ConditionalCommand(onTrue, onFalse, condition);
    }

    public static double wrapAngleRad(double angleRad) {
        double wrapped = angleRad % (2 * Math.PI);
        if (wrapped > Math.PI) wrapped -= 2 * Math.PI;
        if (wrapped < -Math.PI) wrapped += 2 * Math.PI;
        return wrapped;
    }
}