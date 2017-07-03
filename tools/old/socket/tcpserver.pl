#! /usr/bin/perl -w

#tcpserver.pl

use IO::Socket;

$| = 1;

$socket = new IO::Socket::INET (
                                  LocalHost => 'localhost',
                                  #LocalHost => '127.0.0.1',
                                  LocalPort => '5000',
                                  Proto => 'tcp',
                                  Listen => 5,
                                  Reuse => 1
                               );
                                
die "Coudn't open socket" unless $socket;

print "\nTCPServer Waiting for client on port 5000: \n";

while(1)
{
	$client_socket = "";
	$client_socket = $socket->accept();
	
	$peer_address = $client_socket->peerhost();
	$peer_port = $client_socket->peerport();
	
	print "\n I got a connection from ( $peer_address , $peer_port ) ";
	
	
	 while (1)
	 {
		 
		 print "\n SEND( TYPE q or Q to Quit):";
		 
		 $send_data = <STDIN>;
		 chop($send_data); 
		 
		 
		 
		 if ($send_data eq 'q' or $send_data eq 'Q')
		    {
			    
			$client_socket->send ($send_data);
			close $client_socket;
			last;
			}
			
		 else
		    {
			$client_socket->send($send_data);
		    }
		    
		    $client_socket->recv($recieved_data,1024);
		    
		    if ( $recieved_data eq 'q' or $recieved_data eq 'Q')
		    {
			    close $client_socket;
			    last;
		    }
		    
		    else
		    {
			    print "\n RECIEVED: $recieved_data";
		    }
		    
	}
}
                                