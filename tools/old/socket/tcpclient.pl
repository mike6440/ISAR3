#! /usr/bin/perl -w

#tcpclient.pl

use IO::Socket;

$socket = new IO::Socket::INET (
                                  PeerAddr  => '10.0.1.5',
                                  PeerPort  =>  5000,
                                  Proto => 'tcp',
                               )                
or die "Couldn't connect to Server\n";

                              
                          
    
                                                          
while (1)
{
	
    $socket->recv($recv_data,1024);
    
    if ($recv_data eq 'q' or $recv_data eq 'Q')
    {
	    close $socket;
	    last;
    }
    else
    {
	    print "RECIEVED: $recv_data"; 
        print "\nSEND( TYPE q or Q to Quit):";
        
        $send_data = <STDIN>;
        chop($send_data);
              
        
        if ($send_data ne 'q' and $send_data ne 'Q')
        {
	        $socket->send($send_data);
        }    
        else
        {
	        $socket->send($send_data);
            close $socket;
            last;
        }
        
    }
}    
