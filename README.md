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

## Vision troubleshooting

### Symptom

Position on field is wrong as reported by Shuffleboard. Robot is not snapping to correct position when an april tag is clearly in view of the camera.

### Troubleshooting steps

1. Power cycle the robot. Photon Vision can crash if the robot has been turned on for a long time.
2. Check the Photon vision UI (https://photonvision-usb.local:5800 for front, https://photonvision-rear.local:5800 for back).
   1. If the blue web page loads, but you see a yellow video frame, this is another indication that photon vision has crashed. Power-cycle the robot or select "Restart PhotonVision" in the settings panel of the website.
   2. If the blue web page loads, but the camera frame is black, and it doesn't show that it is configured for April Tags (near the top right of the web page), that means PV has lost its configuration. You'll need to restore calibration and config from backups. Zip files are in the `/Competition/CameraSettings` folder of this repository.
   3. If the blue web page loads, but the camera frame is over/under exposed, you will need to update the camera exposure and gain settings. Don't use auto exposure, this is known to crash Photon Vision.
   4. If the website doesn't load at all, make sure that you are connected to the robot network, the robot has power, and the Raspberry Pi running Photon Vision is powered and plugged in to the robot network. The Raspberry Pi can be identified by its black finned heat-sink. It is located near the lower arm motors on either side of the robot - follow the ribbon cables from the camera if necessary.