#! /usr/bin/expect

puts "
Simulate ISAR for test.
"

set loguser 0;		#   test 0-quiet, 1=verbose
log_user $loguser;

global SIM SIMPID SER SERPID

#===========================================================================
# PROCEDURE TO CONNECT OUTPUT ISAR SIMULATION RECORDS
# be sure .kermrc has a line
#   prompt "k>>"
# input
#	infoname = full name for the info file used by write_info().
#	isarport = full path name for the serial port, e.g. /dev/tty.usbserial0
#	simflag = 0/1 for no/yes simulate isar
#	simfile = full name of the simulation file, set in the setup file.
#============================================
proc spawn_kermit_to_isar {} {
	set timeout 10
	if {$simflag == 0 } {
		# START PROCESS -- KERMIT FOR ISAR MODEM
		set isarpid [spawn kermit]
		expect {
			timeout {
				write_info $infoname "isar KERMIT FAILS TO OPEN"
				exit 1
			}
			">>"
		}
		write_info $infoname "ISAR KERMIT IS OPEN., isarport = $isarport"
		
		set timeout 3
		## OPEN THE PORT
		send "set line $isarport\r"
		expect ">>"
		send_user "set line $isarport\n";
		## SPEED
		send "set speed 9600\r"
		## DUPLEX
		send "set duplex half\r"  ;#v02
		expect ">>"
		## FLOW CONTROL
		send "set flow none\r"
		expect ">>"
		## CARRIER WATCH
		send "set carrier-watch off\r"
		expect ">>"
		## CONNECT 
		send "connect\r"
		expect {
			"Conn*---"  {send_user "ISAR CONNECTED\n"}
			timeout {send_user "TIMEOUT, NO CONNECT"; exit 1}
		}
	} else { 
		# PRESENT DIRECTORY
		send_user "SIMULATE ISAR\n";
		set isarpid [spawn perl simulate/isar_simulator.pl]
	}
	set ISAR $spawn_id
	write_info $infoname "ISAR PROCESS ID (PID): $ISAR ($isarpid)";
	return $ISAR
}



exit

