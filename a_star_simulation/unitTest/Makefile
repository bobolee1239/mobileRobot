all: testMapNode testOpenList testRobot testMap

FLAG = -std=c++11 -DDEBUG -g
INCLUDES = ../include/a_star.h

testMapNode: testMapNode.cpp $(INCLUDES)
	g++ $(FLAG) testMapNode.cpp -o testMapNode

testOpenList: testOpenList.cpp $(INCLUDES)
	g++ $(FLAG) testOpenList.cpp -o testOpenList

testRobot: testRobot.cpp $(INCLUDES)
	g++ $(FLAG) testRobot.cpp -o testRobot

testMap: testMap.cpp $(INCLUDES) readcsv.h
	g++ $(FLAG) testMap.cpp -o testMap

clean :
	rm -rf *.o testMapNode testOpenList testRobot testMap
