# $Id: makefile,v 1.8 2010/03/24 13:17:06 stoecker Exp $
# probably works not with all compilers. Thought this should be easy
# fixable. There is nothing special at this source.

ifdef windir
CC   = g++
OPTS = -Wall -W -O3 -DWINDOWSVERSION 
LIBS = -lwsock32
else
OPTS = -Wall -W -O3 
endif

ntripclient: 
	g++ -std=c++11 -c -o ntrip_helper.o ntrip_helper.c -g
	g++ -std=c++11 -c -o serial.o serial.c -g
	g++ -std=c++11 -c -o ntripclient.o ntripclient.c -g
	g++ -std=c++11 -c -o Node.o Node.cpp -g
	g++ -std=c++11 -c -o xml_reader.o xml_reader.cpp -g
	g++ -std=c++11 -pthread -o ntripclient ntrip_helper.o serial.o ntripclient.o xml_reader.o Node.o -g
	#ntrip_helper.c ntripclient.c serial.c
	#g++ $(OPTS) ntripclient.c -o $@ $(LIBS) -std=c++11

clean:
	$(RM) ntripclient core*


archive:
	zip -9 ntripclient.zip ntripclient.c makefile README serial.c

tgzarchive:
	tar -czf ntripclient.tgz ntripclient.c makefile README serial.c
