RPL Border router
====================

This example features a simple RPL border router running on top of Contiki to
bridge the sensor network to Internet.

To run the example on real nodes under Linux
--------------------------------------------

1. Program one node with the RPL border router

    cd ../rpl-border-router && make TARGET=jn5168 border-router
    (flash border-router.jn5168.bin file using NXP flash utility)
    
2. Program other nodes with a RPL-enabled application (i.e., rpl-udp/udp-client)

    cd ../rpl-udp && make TARGET=jn5168 udp-client
		(flash udp-client.jn5168.bin file using NXP flash utility)

3. Connect to the border router using tunslip6:

    make connect-router

		-- it is important to note that the actual utility used to connect the border router to the pc is
		contiki/tools/tunslip6.c
		If you are running under Cygwin, you will need the native gcc to compile it. 
		
4. Reboot the router and note the router IP address (on the console)

5. You should now be able to browse to your router node using your web browser:
   http://[<ROUTER IPv6 ADDRESS>]/. On this page you should see a list of all
   accessible nodes with their IP adresses.
