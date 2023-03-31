# FRC CommonLib Robot Template Project [![Build Status](https://dev.azure.com/Team488/Team%20488%20Builds/_apis/build/status/Team488.TeamXbot2023?branchName=main)](https://dev.azure.com/Team488/Team%20488%20Builds/_build/latest?definitionId=9&branchName=main)

This is a template project which includes all the boilerplate to use our Seriously Common Lib. It can be forked and used as a robot project for the season if you want to use our library.

To find out more about the library, visit the [Seriously Common Lib repo](https://github.com/Team488/SeriouslyCommonLib).

## Up-front warning

This template, like our library as a whole, is a work-in-progress. See the library readme for more information.

# Emergency maintenance playbook

## Swerve CANcoder offset calibration

- open Pheonix tuner x
- Config
- Refresh
- 0 the magnet offset deg
- Apply config
- Go to self-test
- Physically move module so it's pointing forward (xs aligned)
- Refresh
- Read current absolute position, copy value
- Back to Config
- Enter -1 \* the angle you copied
- Apply config
- Back to self-test
- Refresh
- Verify absolute angle now reads 0

## Swerve CANcoder setup

If we have to install a new CANcoder, we need to:

- in competition electrical contract, find the ID the device should have
- open pheonix tuner x
- find the new can coder device (will have a generic name)
  - if you can't find it, it might have a duplicate id with another can coder and the tool will only show you one a time. Ask electric to unplug the other CANCoders
- on details tab
- Update the ID and Name
- Use Swerve debug command to figure out what direction is 'forward' on the wheel and use that to line up the module
  - left stick controls the drive of 1 wheel at a time
  - right stick controls the swerve module rotation
  - next module command will rotate between the 4 modules
- Now do the steps in "Swerve CANcoder offset calibration" above
- Additionally, on config verify that:
  - aboslute sensor range: range_plusminus_180
  - sensor initialization: BootToAbsValue

## Setting up CANSparkMAX speed controllers

- Use the Rev hardware client program
- you need to plug into PDP (power distribution panel) or into the individual device (with USB C)
- Give it the right name + number (from electrical contract)
- make sure the motor type is Rev NEO Brushless