// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.PathFollow.Util;

import java.util.Dictionary;
import java.util.Hashtable;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class TriggerHandler{
    static Dictionary<String,TrigShell> trigDict = new Hashtable<String,TrigShell>();

    public static void set(String key, TrigShell trigger)
    {
        TriggerHandler.trigDict.put(key,trigger); 
    }

    public static TrigShell get(String key){
        return TriggerHandler.trigDict.get(key);
    }

    // public static void whileTrue(String key, Command command)
    // {
    //     TriggerHandler.trigDict.get(key).whileTrue(command);
    // }

    // public static void whileFalse(String key, Command command)
    // {
    //     TriggerHandler.trigDict.get(key).whileFalse(command);
    // }

    // public static void toggleOnFalse(String key, Command command)
    // {
    //     TriggerHandler.trigDict.get(key).toggleOnFalse(command);
    // }

    // public static void toggleOnTrue(String key, Command command)
    // {
    //     TriggerHandler.trigDict.get(key).toggleOnTrue(command);
    // }



    




}
