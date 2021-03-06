#! /usr/bin/perl

#use Term::ReadKey;
use Time::Local;		# v102 use Time module
use Time::gmtime;
use Time::localtime;

#CALLING: call from the main folder which contains the simulate folder.
# default input file::		./simulator/isar6_simulator.pl
# define input file::		./simulator/isar6_simulator.pl  filename

# remove time stamp and insert a new one for now.
#20100527T155706Z,$ISAR5,19700101T000214Z,279.98,0.6861,0.7169,1.7753,1.7714,1.7702,2.3766,2.3750,2.3751,2495,1955,1948,1971,1885,1962,2594,3631,1,0,  1.8,  0.0,355.1, 24.5,-999.000000,-999.00000,-999.0,-999.0,-999.0,300.8,305.9,34,03/4801
#SOES RECORD
#20100609T170836Z,$ISAR5,20100609T170358Z,280.05,0.8751,0.6215,2.2370,2.2359,2.2355,2.8253,2.8237,2.8237,2493,2313,2305,2324,2250,2319,2295,3592,1,0,  1.7, -4.2, 68.1,155000.0,50.811085,-1.09322, 0.0, 0.0, 5.5,292.1,296.6,552,03/4801


if ( $#ARGV >= 0 ) {
	$fin = sprintf "%s", shift();
} else {
	$fin = 'isar.txt';
}
	if( ! -f $fin ) {
	print"SIMULATE FILE OPEN ERROR."; 
	printf"Current path = %s",`pwd`;
	printf"File %s does NOT exist\n",$fin;
	die;
}
print"OPEN SIMULATEFILE $fin\n";

my $irec = 0;
my $i1;

while (1) {
	$irec = 0;
	#print"test Opening data file $fin\n";
	open(F,"<$fin") or die("fin error\n");
	# Loop through all the data records.
	while (<F>) {
		chomp( $str = $_);
		#print"test $str\n";
		# STRIP OFF THE BEGINNING TIME.
		$i1=index($str,'$I');
		$str = substr($str,$i1);
		#print"test i1 = $i1\n";		
		#print"test substr=$str\n";
		
		# IS THIS AN ISAR STRING
		if ( $str =~ /^\$IS/) {
			print"$str\r\n";
			sleep(2);
			$irec++;
		} else {
			# SIMPLY PRINT OUT THE NON-DATA LINE, DO NOT WAIT
			#print "$str\n";
		}
		if ($irec >10){print"EXIT\n"; exit 1}
	}
	print"Starting over\n";
	close F;
}
#*************************************************************/
sub now
{
# Get gmt time which uses the computer's time zone and daylight savings state,
# then use $timezone to put the time into local standard time.
#v101 060629 rmr -- start config control
	my $tm;
	#use Time::gmtime;
	$tm = gmtime;	#gmtime 	# CURRENT DATE 
	my $nowsec = datesec($tm->year+1900, $tm->mon+1, $tm->mday, $tm->hour, $tm->min, $tm->sec);
	return ( $nowsec);
}

#*************************************************************/
sub datesec 
# Convert yyyy,Mm,dd,hh,mm,ss to epoch secs
#v101 060629 rmr -- start config control
#v102 060629 rmr -- add use command
#Note, we use local time so we can get the epoch seconds exactly for the input datetime.
#
{
	#use Time::Local;		# v102 use Time module
	my $dtsec;
	$dtsec = timelocal($_[5], $_[4], $_[3], $_[2], $_[1]-1, $_[0]-1900);
	return ($dtsec);
}

#***************************************************************/
sub dtstr
# Convert epoch second to a time string
# CALL
#	$str = dtstr($dtsec, $format)
# INPUT
#	$dtsec = time since 1970 in secs.  Unix time command
#	$format = 	'long'  yyyy-MM-dd (jjj) hh:mm:ss
#				'short' yyMMdd, hhmmss
#				'jday' just the floating point julian day
#               'prp' suitable for the prp instrument
#               'csv' comma separated variable
#               'scs' same as SCS program
#               'ssv' space separated variables
#				'iso' yyyyMMddThhmmssZ
# v102 060622 rmr start toolbox cfg control
# v103 060627 rmr -- add long and short formats
# v104 100828 rmr -- brought up to date with other version floating around.
# v5 101119 rmr -- get localtime not gmtime.
#v6 110326 rmr -- ISO time yyyyMMddThhmmssZ
{
	my ($tm, $fmt);					# time hash
	my ($str, $n);					# out string
	# use Time::localtime;		# use Time module
	
	$n = $#_;
	$tm = localtime(shift);		# convert incoming epoch secs to hash
	# ==== DETERMINE THE FORMAT TYPE =============
	$fmt = 'long';	
	if ( $n >= 1 ) {  $fmt = shift() }
	
	if ( $fmt =~ /long/i ) {
		$str = sprintf("%04d-%02d-%02d (%03d) %02d:%02d:%02d" ,
			$tm->year+1900, $tm->mon+1, $tm->mday, $tm->yday+1,$tm->hour, $tm->min, $tm->sec);
	}
	elsif ( $fmt =~ /short/i ) {
		$str = sprintf("%04d%02d%02d,%02d%02d%02d" ,
			($tm->year+1900), $tm->mon+1, $tm->mday, $tm->hour, $tm->min, $tm->sec);
	}
	elsif ( $fmt =~ /jday/i ) {
		$str = sprintf("%02d-%03d" , $tm->year-100, $tm->yday+1);
	}
	elsif ( $fmt =~ /prp/i ) {
		$str = sprintf("%02d%02d%02d" , $tm->year-100, $tm->mon+1, $tm->mday);
	}
	elsif ( $fmt =~ /csv/i ) {
		$str = sprintf("%04d,%02d,%02d,%02d,%02d,%02d" ,
			($tm->year+1900), $tm->mon+1, $tm->mday, $tm->hour, $tm->min, $tm->sec);
	}
	# SPACE SEPARATED VARIABLES
	elsif ( $fmt =~ /ssv/i ) {
		$str = sprintf("%04d %02d %02d %02d %02d %02d" ,
			($tm->year+1900), $tm->mon+1, $tm->mday, $tm->hour, $tm->min, $tm->sec);
	}
	elsif ( $fmt =~ /scs/i ) {
		$str = sprintf ( "%02d/%02d/%04d,%02d:%02d:%02d", 
			$tm->mon+1, $tm->mday, ($tm->year+1900), $tm->hour, $tm->min, $tm->sec);
	}
	elsif ( $fmt =~ /iso/i ) {
		$str = sprintf( "%04d%02d%02dT%02d%02d%02dZ", 
			$tm->year+1900, $tm->mon+1, $tm->mday, $tm->hour, $tm->min, $tm->sec);
	}
	
	return ( $str );				# return the desired string
}
