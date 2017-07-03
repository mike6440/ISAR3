package perltools::MRtime;
use Exporter;
@ISA = ('Exporter');
@EXPORT = qw(&now &gmtoffset &datesec &datevec &dt2jdf &dtstrold &dtstr &dtstr2dt &jdf2dt &julday &WaitSec);

#USE IN A PROGRAM
# use lib "/Users/rmr/swmain/perl";
# use perltools::MRtime;
# my $dt = perltools::MRtime::now();
# printf"%s\n", dtstr($dt);


use Time::Local;
#use Time::localtime;
use Time::HiRes;

sub now
{
	my $t = Time::HiRes::time() + gmtoffset();
	my $tr = sprintf"%.0f",$t;
	return $tr;
}

#-----------------------------------------------------------------
sub gmtoffset
#  gmt = local + gmtoffset
{
	my @t = localtime(time());
	my $tg = timegm(@t);
	my $tl = timelocal(@t);
	my $tz = $tg - $tl;
	return $tz;
}



#*************************************************************/
sub datesec 
# Call: $dt = datesec($yyyy,$MM,$dd,$hh,$mm,$ss);
# Convert yyyy,Mm,dd,hh,mm,ss to epoch secs
#use Time::Local;
#use Time::localtime;
# We assume the yyMMddhhmmss are in the current tzone and no corrections are made.
{
	my $dtsec;
	$dtsec = timelocal($_[5], $_[4], $_[3], $_[2], $_[1]-1, $_[0]-1900);
	return ($dtsec);
}
#*************************************************************/
sub datevec
# ($yy, $MM, $dd, $hh, $mm, $ss, $jdf) = datevec($dtsec);
# convert dtsec to yyyy,MM,dd,hh,mm,ss integers
#v101 060629 rmr -- start config control
#v102 060629 rmr -- add use statement, change $tm->hour to $tm->hours, add jdf to output array
#v103 060630 rmr -- fixed jdf output
#v104 071206 rmr -- use gmtime
{
	use Time::localtime;		# v102 use Time module
	my $tm = localtime(shift);
	$jdf = $tm->yday + ( $tm->hour/24 + ($tm->min + $tm->sec/60 ) / 60 ) / 24;
	($tm->year+1900, $tm->mon+1, $tm->mday, $tm->hour, $tm->min, $tm->sec, $jdf+1);  #v102 v103
}
# ****************************************************************
sub dt2jdf
{
 # call with ($yyyy,$jdf) = dt2jdf($dtsecs)
 # from PERL ref $tm = localtime($TIME);     # or gmtime($TIME)
 # input
 #   $dtsecs = epoch seconds as from the timegmt()
 # output array
 #  0  yyyy = year as four digit number
 #  1  year day integer
 #  2  f.p. year day
 # 2006-3-13
 #v101 060629 rmr -- start config control
 #v102 060629 rmr -- add use 
	use Time::localtime;		# v102 use Time module
	my ($tm);
	$tm = localtime($_[0]);
	return (
	    $tm->year+1900, 
	    $tm->yday+1 + $tm->hour/24 + $tm->min/1440 + $tm->sec/86400);
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
#				'date' yyyyMMdd
# v102 060622 rmr start toolbox cfg control
# v103 060627 rmr -- add long and short formats
# v104 100828 rmr -- brought up to date with other version floating around.
# v5 101119 rmr -- get localtime not gmtime.
#v6 110326 rmr -- ISO time yyyyMMddThhmmssZ
#7  111024 rmr -- add date
{
	my ($tm, $fmt);					# time hash
	my ($str, $n);					# out string
	use Time::localtime;		# use Time module
	
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
	elsif ( $fmt =~ /date/i ) {
		$str = sprintf("%02d%02d%02d" ,($tm->year+1900)%100, $tm->mon+1, $tm->mday );
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
#*************************************************************/
sub dtstr2dt
# Convert a time string in any of several formats to epoch 
# seconds.
# CALL: $dt = dtstr2dt($dtstr);
# INPUT
#  $strin -- time string
# OUTPUT
#  $dt 	-- epoch secs corresponding to dtstr.
#
# FORMATS
# 'long'	--	yyyy-MM-dd (jjj) hh:mm:ss
# 'short'	--	yyMMdd,hhmmss  or yyyyMMdd,hhmmss
# 'csv'		--	yyyy,MM,dd,hh,mm,ss
# 'iso'		--	yyyyMMddThhmmssZ
# 'sbd'		-- Wed, 10 Oct 2007 20:17:25 -0400
#   or		-- Wed, 10 Oct 2007 20:17:25 -0400 (EDT)
#
# v101 060628 rmr -- first coding in a03_da0_avg.pl
# v102 060708 rmr -- return 0 if the time string is bad.
# v103 060708 rmr -- extra check of the time string
# v104 060808 rmr -- added International time: yyyyMMddThhmmssZ
# v105 061030 rmr -- add SCS MM/dd/yyyy,hh:mm:ss
# v106 061107 rmr -- modify bad string message
# v7 070325 -- Thu, 22 Mar 2007 19:55:08 -0400
# v8 071109 -- Wed, 10 Oct 2007 20:17:25 -0400 (EDT)
{
# 	use Date::Manip qw(ParseDate UnixDate);
	
	my ($dtstr, $n);
	my $t = 0;
	my @dat;
	my ($dt, $i, $k);
	my @month = qw(Jan Feb Mar Apr May Jun Jul Aug Sep Oct Nov Dec);
	my ($dum);
	
	
	$n = $#_;
	$dtstr = shift();
	$k=0;	
	
	# CHECK FOR A GOOD DATE TIME STRING
	if ( $dtstr =~ /[^0-9(),\/:-]TZ/ ) {  # v104 
		print"dtstr2dt finds bad time string: $dtstr\n";
		return 0;
	}
	
	# ==== SPLIT THE INPUT STRING INTO PARTS ====
	@dat = split ( /[,\/ ():]+/, $dtstr );
	$k = $#dat;
	#print"dtstr=$dtstr\n";
  	#$i=0; foreach $wd (@dat) {print"$i  $wd\n"; $i++}
  	#die;
	
	
	#====== #v8 Wed, 10 Oct 2007 20:17:25 -0400 (EDT) ==========
	# REMOVE THE LAST WORD (EDT)
	if ($#dat == 8 && $dat[8] =~ /(...)/){
		$dum = pop(@dat);
		#print"dat size was 8, now it is $#dat\n";
	}
  	
	#=====================================================
	# "Thu, 22 Mar 2007 19:55:08 -0400"
	#   0   1   2   3   4  5  6    7
	if ($#dat == 7 ) {
		@dat = split ( /[,\/ ():]+/, $dtstr );  # redo but allow minus sign
# 		if ($k==8) {
# 			print"Reading email date format, $dtstr\n";
# 		}
		for ($i=0; $i<=12; $i++) {
			if( $dat[2] =~ /$month[$i]/ ) {
# 				print"Month = $month[$i], $i\n";
				$dat[2] = $i + 1;
				last;
			}
		}
		## ERROR IN THE TIME FORMAT
		if ( $i == 12 ) { 
			die("dtstr2dt error: i = $i, month(i) = $month[$i]  string[2] = $dat[2], , Bad month in email date.\n");
		}
		$dt[0] = $dat[3];
		if ( length($dat[3]) <= 2 ) { $dt[0] += 2000 }
		$dt[1] = $dat[2];
		$dt[2] = $dat[1];
		$dt[3] = $dat[4];
		$dt[4] = $dat[5];
		$dt[5] = $dat[6];
		$dt[6] = int($dat[7] / 100) + ($dat[7]%100)/60;
		$t = datesec( $dt[0], $dt[1], $dt[2], $dt[3], $dt[4], $dt[5] ) - $dt[6]*3600;
# 		if ($k==8) {
# 			printf"dt = @dt, Timezone = %.3f\n",$dt[6];
# 			printf"Final GMT: %s\n", dtstr($t);		
# 			die;
# 		}
		return $t;
	}
	
	if ( $#dat == 6 ) {
		# ==== LONG FORMAT =================
		# yy-MM-dd (jjj) hh:mm:ss or yyyy-MM-dd (jjj) hh:mm:ss
		$dt[0] = $dat[0];
		if ( length($dat[0]) <= 2 ) { $dt[0] += 2000 }
		$dt[1] = $dat[1];
		$dt[2] = $dat[2];
		$dt[3] = $dat[4];
		$dt[4] = $dat[5];
		$dt[5] = $dat[6];		
	} elsif ( $#dat == 5 ) {
		# ==== CSV FORMAT =====================
		# MM/dd/yyyy hh:mm:ss
		if ( length($dat[2]) >= 4 ) {
			$dt[0] = $dat[2];
			$dt[1] = $dat[0];
			$dt[2] = $dat[1];
			$dt[3] = $dat[3];
			$dt[4] = $dat[4];
			$dt[5] = $dat[5];		
		}
		# 2006/MM/dd hh:mm:ss  or 06/MM/dd hh:mm:ss
		# yyyy,MM,dd,hh,mm,ss  or yy,MM,dd,hh,mm,ss
		else {
			$dt[0] = $dat[0];
			if ( length($dat[0]) <= 2 ) { $dt[0] += 2000 }
			$dt[1] = $dat[1];
			$dt[2] = $dat[2];
			$dt[3] = $dat[3];
			$dt[4] = $dat[4];
			$dt[5] = $dat[5];		
			#print"ssv format: @dat\n";
			#print"dt: @dt\n";
		}
	} elsif ( $#dat == 1 ) {
		# === SHORT FORMAT ===================
		if ( length($dat[0]) == 6 ) {
			$dt[0] = substr($dat[0],0,2) + 2000;
		} else { $dt[0] = substr($dat[0],0,4) }
		$dt[1] = substr($dat[0],-4,2);
		$dt[2] = substr($dat[0],-2,2);
		$dt[3] = substr($dat[1],0,2);
		$dt[4] = substr($dat[1],2,2);
		$dt[5] = substr($dat[1],4,2);
	} elsif ($#dat == 0 && substr($dat[0],8,1) eq 'T' ) {
		# ==== ISO STANDARD yyyyMMddThhmmssZ ==================  v104
		$dt[0] = substr($dat[0],0,4);
		$dt[1] = substr($dat[0],4,2);
		$dt[2] = substr($dat[0],6,2);
		$dt[3] = substr($dat[0],9,2);
		$dt[4] = substr($dat[0],11,2);
		$dt[5] = substr($dat[0],13,2);
		#print"@dt\n";
	} else {
		print"dtstr2dt finds unknown time string format: <<$dtstr>>\n";  #v106
		die 'test1';
		return 0;
	}
	#print"datesec call: @dt\n";
	$t = datesec( $dt[0], $dt[1], $dt[2], $dt[3], $dt[4], $dt[5] );
	return $t;
}

#*************************************************************/
sub jdf2dt
{
# jdf2dt(yyyy, jdf) -->> EpochSecs
# Actually, jdf is the f.p. yearday which = 1 on 1 Jan.
# 2003-3-13
#v101 060629 rmr -- start config control
	my $yyyy = $_[0];
	my $jdf = $_[1];

	# FIND ESECS ON 1 JAN
	my $esecs = timegm(0,0,0,1,0,$yyyy-1900);
	
	# ADD ON THE CURRENT JDAY
	$esecs += ($jdf - 1) * 86400;
	#print "esecs at jdf = $jdf: $esecs\n";	
	return $esecs;
}
#************************************************************/
# julday.pl  subroutine
#Compute the Julian Day per "Numerical Recipies in C"
#v101 060629 rmr -- start config control
sub julday
{
	#use POSIX;
	my($iyyy) = shift();  
	my($MM) = shift();  
	my($dd) = shift();;
	#printf("\njulday: %4d-%02d-%02d\n", $iyyy, $MM, $dd);

	my($igreg) = 588829; #(15+31*(10+12*1582));
	#print"igreg = $igreg\n";
	
	$jy = $iyyy;
	#printf"jy = $jy\n";
	
	# CORRECTIONS AND CHECKS
	if ($jy == 0) {die("julday: there is no year zero.");}
	if ($jy < 0) { ++$jy};
	if ($MM > 2) {
		$jm = $MM + 1;
	} else {
		--$jy;
		$jm = $MM + 13;
	}
	#printf"jm = $jm,   jy = $jy\n";
	
	# NOTE: THE INT FUNCTIONS NEED TO BE CHANGED FOR FLOOR FOR jy < 0
	$jul = floor(365.25 * $jy) + floor(30.6001 * $jm)
		+ $dd + 1720995;
	#printf("  jul = %d\n", $jul);
	
	# SMALL CORRECTION
	if ( $dd + 31 * ($MM + 12 * $iyyy) >= $igreg) {
		$ja = floor(0.01 * $jy);
		$jul += 2 - $ja + floor(0.25 * $ja);
	}
	#printf("julday = %d\n",$jul);
	return($jul);
}

#=======================================================
sub WaitSec {
	
	my $waitsec = shift();
	my ($then, $now);
	$then = $now = perltools::MRtime::now();
	while ( $now - $then < $waitsec ) {
		$now = perltools::MRtime::now();
	}
	return;
}


1;
