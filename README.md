rc_balance_ethan, written by Ethan Foss using robotcontrollib.

Description: Program to Balance Motorized Inverted Pendulum using two lead-lag controllers in a 
successive loop closure fashion. Program will run and initialize orientation, and following a short
cooldown, the MIP can be uprighted to start the controllers. State data for the MIP is printed in the
command window, and the MIP can be disarmed by tipping it over.

Use: Plug in beagle bone, connect to beaglebone with filezilla using the following information:

Host: sftp://192.168.7.2
Username: debian
Password: temppwd
Port: [blank]

If the folder ethancontrol exists in the MIP already, then no other steps are necessary and filezilla
may be disconnected. Otherwise, upload the folder ethancontrol to the debian folder.

Open putty, and connect by typing in the IP address: 192.168.7.2. Then, connect to the MIP with the 
following infromation:

Username: debian
Password: temppwd

Change directory to ethancontrol with the following command:

>> cd ethancontrol

Make the file using the following command:

>> make rc_balance_ethan

Then run the program with the following command:

>> ./rc_balance_ethan

If desired, the program can be edited directly from the terminal with the following command:

>> nano rc_balance_ethan.c
