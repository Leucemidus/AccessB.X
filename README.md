# AccessB.X
MPLABX XC8 project for AccessB hardware PIC18F2550

IMPORTANT: This project will compile without problems with XC8 v1.32, later versions have changes on plib files so it will not compile, may be in a future I will update the project to compile with newer XC8 versions. You can download the legacy v1.32 of XC8 and install it, then in the project properties change the compiles from the XC8 version that you have installed to the 1.32, apply changes and compile.

This is the folder project for MPLABX with XC8, to make this project compile you must edit the MPLABX project properties under
XC8 global options select "XC8 compiler" and you must set the project directory address in the "include directory" field, then apply the
changes and clean and build the project it must compile without error.
