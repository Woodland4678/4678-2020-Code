/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "frc/commands/Command.h"
#include "frc/commands/Subsystem.h"
#include "Subsystems/PathFinder/Path.h"
#include "Robot.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class Slalom
    : public frc::Command {
 public:
  Slalom();

  void Initialize() override;

  void Execute() override;

  void End() override;

  bool IsFinished() override;

  PathFinder *path1;
  PathFinder *path2;
  PathFinder *path3;
  int autoStep = 0;
  double rVel = 0;
	double lVel = 0;
  bool done = false;
};
