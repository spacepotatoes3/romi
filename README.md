Adapted from VS Code romi example to run a vision routine

Vision Routine (written in autonomousPeriodic):
- Runs autonomous routine (autonomousSearch) to search for apriltags repeatedly (currently spinning 360 degrees)
- Repeatedly checks a value written to NetworkTables by a vision coprocessor to see if a tag has been detected
- When a tag is detected, cancels all commands and runs an autonomous routine (autonomousVision)
