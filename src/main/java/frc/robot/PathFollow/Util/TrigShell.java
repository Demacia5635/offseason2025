// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.PathFollow.Util;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class TrigShell extends Trigger{

    Command whileFalse;
    Command whileTrue;
    Command onTrue;
    Command onFalse;
    Command toggleOnTrue;
    Command toggleOnFalse;

    double deb_time; //seconds
    DebounceType deb_type;

    public TrigShell(BooleanSupplier cond)
    {
        super(cond);

    }

    @Override
    public TrigShell debounce(double seconds,Debouncer.DebounceType type)
    {
        BooleanSupplier cond = new BooleanSupplier() {
          final Debouncer m_debouncer = new Debouncer(seconds, type);

          @Override
          public boolean getAsBoolean() {
            return m_debouncer.calculate(this.getAsBoolean());
          }
        };
        TrigShell trig = new TrigShell(cond);
        trig.deb_time = seconds;
        trig.deb_type = type;

        return trig;
    }
    
    @Override
    public TrigShell debounce(double seconds)
    {
        return debounce(seconds,DebounceType.kRising);
    }

    @Override
    public TrigShell negate()
    {
        TrigShell trig = new TrigShell(() ->  !this.getAsBoolean() );

        return trig;
    }

    @Override
    public TrigShell and(BooleanSupplier cond)
    {
        TrigShell trig = new TrigShell(() ->  cond.getAsBoolean() && this.getAsBoolean() );

        return trig;
    }

    @Override
    public TrigShell or(BooleanSupplier cond)
    {
        TrigShell trig = new TrigShell(() ->  cond.getAsBoolean() || this.getAsBoolean() );

        return trig;
    }

    @Override
    public TrigShell whileFalse(Command command)
    {
        super.whileFalse(command);
        this.whileFalse = command;
        return this;
    }

    @Override
    public TrigShell whileTrue(Command command)
    {
        super.whileTrue(command);
        this.whileTrue = command;
        return this;
    }

    @Override
    public TrigShell onFalse(Command command)
    {
        super.onFalse(command);
        this.onFalse = command;
        return this;
    }

    @Override
    public TrigShell onTrue(Command command)
    {
        super.onTrue(command);
        this.onTrue = command;
        return this;
    }

    @Override
    public TrigShell toggleOnTrue(Command command)
    {
        super.toggleOnTrue(command);
        this.toggleOnTrue = command;
        return this;
    }

    @Override
    public TrigShell toggleOnFalse(Command command)
    {
        super.toggleOnFalse(command);
        this.toggleOnFalse = command;
        return this;
    }

}
