#! /usr/bin/perl -w

use IO::Socket;
my $sock = new IO::Socket::INET (
                                 PeerAddr => '10.0.1.4',
                                 PeerPort => '7070',
                                 Proto => 'tcp',
                                );
die "Could not create socket:\n" unless $sock;
print $sock "Hello there\r\n";
close($sock);
