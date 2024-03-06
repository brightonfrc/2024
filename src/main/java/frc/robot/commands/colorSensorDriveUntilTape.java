// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.I2C;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.util.Color;

public class colorSensorDriveUntilTape extends Command {
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private Color tileColor;

  public colorSensorDriveUntilTape(Color colorsToDetect[], Color tileColor) {
    m_colorMatcher.addColorMatch(tileColor);
    this.tileColor = tileColor;
    for (Color colorToDetect : colorsToDetect) {
      m_colorMatcher.addColorMatch(colorToDetect);
    }
  }


  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Color detectedColor = m_colorSensor.getColor();
    int proximity = m_colorSensor.getProximity();
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    if (match.color == tileColor) {
      // put start code code here
    } else {
      // put stop code here
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
