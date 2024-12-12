// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Chassis.utils;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utils.CancoderConfig;
import frc.robot.utils.TalonConfig;

/** Add your docs here. */
public class ModuleConstants {
    String name;
    public TalonConfig driveConfig;
    public TalonConfig steerConfig;
    public CancoderConfig cancoderConfig;
    public final Translation2d moduleTranslationOffset;
    public ModuleConstants(String name, TalonConfig driveConfig, TalonConfig steerConfig, CancoderConfig cancoderConfig, 
      Translation2d moduleTranslationOffset) {
        this.name = name;
        this.driveConfig = driveConfig;
        this.steerConfig = steerConfig;
        this.cancoderConfig = cancoderConfig;
        this.moduleTranslationOffset = moduleTranslationOffset;
    }
}
