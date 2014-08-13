The Contiki Operating System
============================

[![Build Status](https://secure.travis-ci.org/contiki-os/contiki.png)](http://travis-ci.org/contiki-os/contiki)

Contiki is an open source operating system that runs on tiny low-power
microcontrollers and makes it possible to develop applications that
make efficient use of the hardware while providing standardized
low-power wireless communication for a range of hardware platforms.

Contiki is used in numerous commercial and non-commercial systems,
such as city sound monitoring, street lights, networked electrical
power meters, industrial monitoring, radiation monitoring,
construction site monitoring, alarm systems, remote house monitoring,
and so on.

For more information, see the Contiki website:

[http://contiki-os.org](http://contiki-os.org)

JN5168 Port - Alpha
===================

It is important to note that this port (under 'platform/jn5168') provides
only basic functionality, such as:
* UART/printf
* Radio
* Leds
* IPv6 stack 
* Rime stack
* Null mac / Null RDC

In order to build an application for jn5168 you need to copy the file 
'platform/jn5168/App_Stack_Size.ld' to your project directory and 
modify it according to your project's stack needs.

Disclaimer 
==========

This is a work in progress as part of a project with an ongoing development for 802.15.4e TSCH
* The code might not compile and is unstable. You have been warned.
* We do not guarantee anything, plus we will not provide support at the mean time. Use at your own risk.
* The code is the property of the respective authors/organizations metnioned in the heading of each file.
