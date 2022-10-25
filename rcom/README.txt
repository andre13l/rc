INSTRUCTIONS FOR SERIAL PORT PROTOCOL
=====================================

This folder contains the base code of the serial port protocol.

Project Structure
-----------------

- bin/: Compiled binaries.
- src/: Source code for the link_layer implementation.
- include/: Header files of the link_layer file containing structures used throughout the code.
- cable/: Virtual cable program used to test the serial port..
- main.c: Main file containing the application_layer.
- Makefile: Makefile to build the project and run the application.
- penguin.gif: File that is gonna be sent and used for testing.

How to Run the Project
-------------------------------

1. Edit the source code in the src/ directory.
2. Compile the application using the provided Makefile.
3. Run "$ make run_cable" to execute the virtual cable program
4. Test the protocol by running "$ make run_nx" on one of the machines followed by "$ make run_tx".
5. Check if the files received and sent are the same by running "$ make check_files".
5. Test the protocol with cable disconnections and noise by running the receiver and the transmitter, followed by typing "noise" or "disconnect" to add noise or disconnect the cable.
6. Check if the files received and sent match.
