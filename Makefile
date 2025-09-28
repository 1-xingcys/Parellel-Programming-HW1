CXX = g++
CXXFLAGS = -std=c++17 -O2 -pthread -Xpreprocessor -fopenmp -I/opt/homebrew/opt/boost/include -I/opt/homebrew/opt/libomp/include -I/opt/homebrew/opt/tbb/include
LDFLAGS = -L/opt/homebrew/opt/boost/lib -L/opt/homebrew/opt/libomp/lib -L/opt/homebrew/opt/tbb/lib -lomp -ltbb
TARGETS = hw1

.PHONY: all
all: $(TARGETS)

.PHONY: clean
clean:
	rm -f $(TARGETS)
