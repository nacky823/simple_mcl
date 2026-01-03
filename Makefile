CXX := g++
CXXFLAGS := -std=c++17 -O2 -Wall -Wextra -Iinclude

BIN := simple_mcl
SRCS := src/main.cpp
OBJS := $(SRCS:.cpp=.o)

all: $(BIN)

$(BIN): $(OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(BIN) $(OBJS)

.PHONY: all clean
