CXX=g++
CXXFLAGS=-std=c++17 -O2 -Wall -Wextra -Iinclude
planner: src/main.cpp include/*.hpp
	$(CXX) $(CXXFLAGS) -o $@ src/main.cpp
run: planner
	./planner
clean:
	rm -f planner planner.exe *.svg
