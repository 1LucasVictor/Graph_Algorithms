
ALL: main

main: graph.o  main.o dotGenerator.hpp
	c++ main.o graph.o -o main

graph.o: graph.cpp
	c++ -c graph.cpp
main.o: main.cpp
	c++ -c main.cpp

clean:
	rm *.o main *.dot