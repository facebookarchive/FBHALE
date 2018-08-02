General
-------
FBHALE was developed by Facebook to aid in the conceptual design of HALE
aircraft. The physics modeled are specifically tailored to tackle many of
the challenges encountered during HALE aircraft design.

FBHALE is written in MATLAB, files are included to compile the code to
binaries.

To date FBHALE has been run on the following operating systems:
MAC_OS
Windows 10
Ubuntu

Additional Required Software
----------------------------
XROTOR  
XFOIL  
ASWING*

Software marked with a "*" requires a special build process described
below.

*Custom ASWING Compilation Steps
The steps below outline changes that must be made when complying ASWING to
ensure proper interface with FBHALE.

ASWING Source Mods:
-------------------------------
The following edits to ASWING's source files are necessary to aid in the
integration of ASWING with the optimization framework. Version 6.02 is
assumed. Line numbers are mentioned with respect to the unmodified source
files.  

1. Increase available number of sensors.
   In DIMEN.INC, line no: 19, set NSENX = 60. This setting ensures that
enough sensors are available for the required structural discretization.

2. Output stability derivative with respect to deflected CG.
   In io.f, after the ENDDO statement in line no: 2056 of subroutine DUMPDM
   (LU, IPNT), enter the following lines:
      XYZREF(1,1) = RCGXYZ(1,IPNT)
      XYZREF(2,1) = RCGXYZ(2,IPNT)
      XYZREF(3,1) = RCGXYZ(3,IPNT)

      XYZREF(1,2) = RCGXYZ(1,IPNT)
      XYZREF(2,2) = RCGXYZ(2,IPNT)
      XYZREF(3,2) = RCGXYZ(3,IPNT)

      XYZREF(1,3) = RCGXYZ(1,IPNT)
      XYZREF(2,3) = RCGXYZ(2,IPNT)
      XYZREF(3,3) = RCGXYZ(3,IPNT)

3. Change display precision for t, y, z, f_aero after eXec operation.
   In io.f, inside subroutine DUMPIT (LU, IPNT) in line nos: 1601-1602,
   for t precision, set:
      NWID(0) = 9
      NDEC(0) = 4

    Similarly, for x,y,z, in line nos: 1606-1611, set:
      NWID(1) = 9
      NWID(2) = 9
      NWID(3) = 9
      NDEC(1) = 4
      NDEC(2) = 4
      NDEC(3) = 4

     and for f_aero, in line nos: 1689-1690, set:
      NWID(IQFL) = 9
      NDEC(IQFL) = 4

Set Up
------
If running on Windows, XROTOR, XFOIL and ASWING must be on your system path. If
running on MacOS, XROTOR, XFOIL and ASWING must be in your Matlab path.

Test system set up by running /Tools/CheckSystem.m.

Documentation/Theory
--------------------
Included in the /Docs file are two papers presented at AIAA Aviation 2018
that highlight the physical modeling, the sizing philosophy for each
subsystem and the code architecture.

 Docs/PartI_SolarPoweredSingleandMultipleBoomAircraft.pdf  
 Docs/PartII_SolarPoweredFlyingWingAircraft.pdf

Example Cases
-------------
The Runs/ folder contains example aircraft and inputs that can be run by
running the provided .m files. The example .m files contain the
optimization variables. For each .m file in the runs folder there is a
corresponding subfolder of the same name that contains the specific inputs
for that example.

The code must be executed from the top level directory with all folders and
subfolders added to the path. As the code runs printouts are displayed to
the Command Window providing some insight into what step in the code is in
and how the aircraft sizing is progressing.

Example Run Sequence:

>> SingleBoom.m

Code Overview
-------------
User specified inputs for each configuration are set in the corresponding
subfolder in /Runs. Please see the example cases for further information on
user specified configuration specific inputs.

Aircraft data is stored in a structure array called "optim" with fields
that hold the details describing the various aircraft subsystems and
quantities.

All aircraft components are categorized as beams or a point mass. The
specification of the various beams and point masses is set in
Runs/__/DesignInputs.m.

All beams must have a structural concept and properties defined,
number of instances (.N) defined and the location defined (.x_m, .y_m).
Beam structural properties, including mass, are stored in the .structure
subfield. Using these inputs the structural properties, mass, and .xCG of
each beam is found. Each beams field name must be added to
optim.aircraft.beamNames in DesignInputs.m.

All point masses must have .mass_kg specified, number of instances (.N)
defined and a .xCG, .y_m and .zCG location defined or subsequently set /
updated. The field name for each point mass must be added to
optim.aircraft.pointMassNames in DesingInputs.m.

The number of instances of a given entity is specified by the field .N. .N
should take the form of a vector with each index representing a different
spanwise location. Similarly each entity should have a .y_m
specified in the same format. Only positive spanwise locations need to be
specified, entities with non zero .y_m and .N > 1 are automatically
reflected to -1*y_m.

All entity properties should follow this spanwise location based vector
format.

Ex.:
optim.propulsion.N       = [1 2]
optim.propulsion.y_m     = [0 10]
optim.propulsion.mass_kg = [10 10]
optim.propulsion.xCG_m   = [0 2.5]

The aircraft CG is computed in UpdateMassProperties.m which computes the
CG of all beams and point masses using their .mass_kg and location. The
placement of various masses is done in a configuration dependent file named
UpdateMassLocations.m located in appropriate subfolder in Runs.

## License

By contributing to FBHALE, you agree that your contributions will be
licensed under the LICENSE or COPYING file in the root directory of this
source tree.
