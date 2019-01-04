ntripclient: 
	mkdir -p build
	g++ -std=c++11 -c -o build/NMEAData.o NMEAData.cpp -g
	g++ -std=c++11 -c -o build/ntrip_helper.o ntrip_helper.c -g
	g++ -std=c++11 -c -o build/serial.o serial.c -g
	g++ -std=c++11 -c -o build/ntripclient.o ntripclient.c -g
	g++ -std=c++11 -c -o build/Node.o Node.cpp -g
	g++ -std=c++11 -c -o build/xml_reader.o xml_reader.cpp -g
	g++ -std=c++11 -pthread -o ntripclient build/ntrip_helper.o build/serial.o build/ntripclient.o build/xml_reader.o build/Node.o build/NMEAData.o -g

clean:
	@[ ! -e build/ ] || rm -rf build
	@[ ! -e ntripclient ] || rm -f ntripclient

archive:
	zip -9 ntripclient.zip ntripclient.c makefile README serial.c

tgzarchive:
	tar -czf ntripclient.tgz ntripclient.c makefile README serial.c
