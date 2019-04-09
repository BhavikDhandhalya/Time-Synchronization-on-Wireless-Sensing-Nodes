1. base station folder contains base staion file(BaseStationP.nc)
2. Blink to radio folder contains cluter member file(BlinkToRadioC.nc)

If you are familiar with the basic method of how to compile and run the code, then you can directly use bash scripts to run or compile the file.
-------------
| PRO TIPS: |
-------------
1. type "sh cmp.sh" to compile the file.
2. type "sh run.sh" to intall the file on the mote. (if you want to change the id of mote then open run.sh file using vim or any other editor and change it.)
3. run.sh file in the base station folder contains the "java listen method which will automatically print packet structure". (Make sure that base station is connected to PC via USB port of PC).

-----------------
| BASIC METHOD: |
-----------------

1. make a package (for telosb platform)
		$ cd directory/
		$ make telosb
		
2. install a package on a node
	$ cd package_directory/
	$ make telosb install,1		// 1 will become node_id
	
3. print messages received by basestation on terminal
	$ java net.tinyos.tools.MsgReader -comm serial@/dev/ttyUSB0:telosb BlinkToRadioMsg
	
	- the usb address changes everytime you reinsert a mote
	- has to be run from the package which contains the .java and .class files
	
4. save terminal output to a file
	java net.tinyos.tools.MsgReader -comm serial@/dev/ttyUSB1:telosb BlinkToRadioMsg > data.txt
	
5. change format of the messages printed on terminal or saved to file
	
	make changes in the BlinkToRadioMsg.java file
		$ sudo gedit BlinkToRadioMsg.java	
	delete the existing BlinkToRadioMsg.class file
	complie the package
		$ make telosb cc2420x	
		
6. Changing resolution of PacketTimeStamp to TMicro
			all nodes need to be using the same resolution of PacketTimeStamp otherwise they will not communicate
			In module file, "uses interface PacketTimeStamp<TMicro,uint32_t>"
				and compile using 		$ make telosb cc2420x

	ERRORs
	1. 
	serial@/dev/ttyUSB0:115200 died - exiting (java.io.IOException: Could not open /dev/ttyUSB0: TOSComm JNI library runtime error: open: No such file or directory)
	
	- wrong USB port number in the command	
	- check with $motelist
	
	2. screen resolution suddenly changes to 640x800
		update virtual box and guest additions and extention pack from the virtual box site.
		
	3. Synchronization error during installing code on telosb
		try with different usb port
		
		
	******************* NOTE: *********************
	The TIMER_PERIOD_MILLI and AM_UNICAST_ADDR for the cluster member nodes (BlinkToRadio code) are defined in header file.
		The BaseStation node must always have same nodeid as the AM_UNICAST_ADDR defined in the header file of the main member node.
	The nodeid of the cluster members must be according to AM_UNICAST_ADDR defined in the cluster head node program.
	
	In the traffic clusters, 
		the cluster members will have code (TrafficClusterNew: BlinkToRadio) with different UNICAST_ADDR. The code has been modified to not respond to Third party node.
		the cluster head will have code (TrafficClusterNew: BaseStation) with the timer period set to obtain desired traffic(changed from 20ms to 40ms). the number of packets(17) can be changed to adjust the duration of the traffic.
		
	All experiments had traffic during data collection and no traffic during testing phase. 
	Testing phase everywhere refers to the collection of third party timestamp data.
	
	current hardcoded node-ids: 
		BaseStation: 40 
		ThirdParty node: 35
		main cluster: 1,2,3,4,5
	
	a one-shot timer is added in the members of main cluster for the Testing phase. This delay is to reduce the collision and loss of packets due to all nodes sending messages to base station at same time.
		Each member node of main cluster should have a different value of this delay.
		
	in testing phase,
		data collected by the basestation: the ith packet received by basestation corresponds to (i+1)th numbered packets coming form member nodes.
		
		
		
Mote 30 : loose connection. resets on touch.
Mote 17: loose battery connection. works fine on usb
Mote 35: code burns but doesnt run
Mote 32: doesnt get programmed
