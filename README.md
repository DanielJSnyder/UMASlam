# UM::Autonomy's SLAM Repo
# Written by Dan Snyder in Fall 2016 but he doesn't want your emails
# Taken over by Benny Johnson (bennyaj@umich.edu) in Winter 2017, I welcome your emails
# Read the paper for a technical overview of this system before you get into the code
## Overview of the top level folders
- bin: The location all of the executables are placed
- lcmtypes: the location of all the lcmtypes (.lcm and the generated files)
- obj: Location all of the .o files are placed
- report: Location of the .tex and other files used to generate the pdf that describes this repo and the research project it was part of.  This describes some of the context surrounding the decisions made in the initial repo.
- unit\_tests: Location for SLAM related unit tests
- src: The location of the source files

## Setting up the repo
Necessary packages
- lcm from April Lab at the University of michigan (lcm-proj on github)
- CMake 3.1 (for building lcm)
- libsfml2-dev

## Build System
The build system is based on makefiles.  Follow the examples provided and reference the make documentation for more information on adding to the build system.
Always compile with optimizations, "make clean optimized" is generally a good default command to run

## Style guidelines
These guidelines should match the UMA repo guidelines

- Opening braces on the same line, closing braces on a new line
- Loops and If statements have a space before and after the parentheses
- Single space between operands, "x = (y + z) / w;" for example
- Two spaces instead of tabs, make sure you set this in your editor
- snake\_case\_variable\_names
- CAPATALIZED\_CONSTANTS
- lowerCamelCase functionNames
- Capital ClassNames with no underscores

## Maintenance guidelines

- Comment about the purpose and context of code, not obvious functionality
- The only thing worse than no comments is incorrect comments, if you change the code, please change the comments
- If something confuses you and you figure it out, write a comment about it to help people in the future
- Keep file specific comments with the code, this README is for general information only
- There are a lot of things that will seem wrong or broken at first, think hard and get a second opinion before changing them
- You can tune most of the important variables in SLAM, try to tweak the constants before you resort to editing the source code
- If you're the maintainer, look for a successor and teach them early, DON'T LEAVE THE PROJECT WITHOUT A MAINTAINER

