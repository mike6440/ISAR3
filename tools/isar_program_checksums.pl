#!/usr/bin/perl -w
# use strict;
# use warnings;

# perl isar_program_checksums.pl /media/rmrco/TD8

if (-d "$ENV{HOME}/Dropbox"){$swpath="$ENV{HOME}/Dropbox/swmain/apps/ISAR3/sw"}
else {$swpath="$ENV{HOME}/swmain/apps/ISAR3/sw"}

print"swpath = $swpath\n";

my ($archivepath) = @ARGV;
if( not defined $archivepath ){
	print"ERROR -- No archive drive specified\n";
	exit;
}
$archivepath="$archivepath/swmain/apps/ISAR3/sw";
$archiveflag=1;
if(! -d $archivepath){
	print"archive path: $archivepath MISSING\n";
	$archiveflag=0;
}

$pgms='ArchiveIsar
avggps
avgisar
avgisarcal
Cleanupisar
ClearIsarData
DaqUpdate
FindUSBPort
fixed_gprmc
getsetupinfo
help.txt
kerm232
kermss
KillScreen
LastDataFolder
LastDataRecord
LastDataTime
PrepareForRun
sbd_transmit
SetDate
sockrx
term_to_gps
term_to_isar
term_to_sbd
UpdateDaq
Z_gps
Z_isar
Z_isarcal
setup/su1.txt
setup/su1c.txt
setup/su4.txt
setup/su4c.txt
../tools/bashrc_add_to_existing.txt
../tools/bashrc_isar3.txt
../tools/crontab_isar3.txt
../tools/kermrc_isar3.txt
../tools/screenrc_isar.txt
../tt8/isar_v13_160605.c';

#print"$pgms\n";
@p=split /\n/,$pgms;

print"  n     sw      archive  File\n";
# 0	29540	29540	ArchiveRosr

$i=0; foreach $f (@p){	
	$e=' ';
	# SW FOLDER
	$ff="$swpath/$f";
	if(! -f $ff){
		#print"file $ff missing\n";
		$s='-9999';
		$e='*';
	} else {
		@f=split / /,`sum $ff`;
		$s=$f[0];
	}
	# ARCHIVE FOLDER
	$ff="$archivepath/$f";
	if(! -f $ff){
		#print"archive file $ff missing\n";
		$e='*';
		$sa='-9999';
	} else {
		@f=split / /,`sum $ff`;
		$sa=$f[0];
	}
	if($sa ne $s){$e='*'}
	if($archiveflag==1){printf"%s%2d\t%d\t%d\t%s\n",$e,$i,$s,$sa,$f}
	else{printf"%d\t%d\t%s\n",$i,$s,$f}
	$i++;
}
