all : genvrp

CCC = g++
# CCC = clang++-10
CCFLAGS = -O3 -Wall -std=c++2a
# CCFLAGS = -Wall -std=c++2a -g
TARGETDIR=.

OBJS2 = \
        $(TARGETDIR)/Genetic.o \
        $(TARGETDIR)/Individual.o \
        $(TARGETDIR)/LocalSearch.o \
        $(TARGETDIR)/main.o \
        $(TARGETDIR)/Params.o \
        $(TARGETDIR)/Population.o \
        $(TARGETDIR)/Split.o

$(TARGETDIR)/genvrp: $(OBJS2)
	$(CCC) $(CCFLAGS) -o $(TARGETDIR)/genvrp $(OBJS2)
	
$(TARGETDIR)/Genetic.o: Genetic.h Genetic.cpp
	$(CCC) $(CCFLAGS) -c Genetic.cpp -o $(TARGETDIR)/Genetic.o

$(TARGETDIR)/Individual.o: Individual.h Individual.cpp
	$(CCC) $(CCFLAGS) -c Individual.cpp -o $(TARGETDIR)/Individual.o

$(TARGETDIR)/LocalSearch.o: LocalSearch.h LocalSearch.cpp
	$(CCC) $(CCFLAGS) -c LocalSearch.cpp -o $(TARGETDIR)/LocalSearch.o
	
$(TARGETDIR)/main.o: main.cpp
	$(CCC) $(CCFLAGS) -c main.cpp -o $(TARGETDIR)/main.o

$(TARGETDIR)/Params.o: Params.h Params.cpp
	$(CCC) $(CCFLAGS) -c Params.cpp -o $(TARGETDIR)/Params.o

$(TARGETDIR)/Population.o: Population.h Population.cpp
	$(CCC) $(CCFLAGS) -c Population.cpp -o $(TARGETDIR)/Population.o

$(TARGETDIR)/Split.o: Split.h Split.cpp
	$(CCC) $(CCFLAGS) -c Split.cpp -o $(TARGETDIR)/Split.o

test: genvrp
	./genvrp ../../../VRPTWController/Instances/Homberger/C1_2_1.txt C1_2_1.sol -seed 1 

clean:
	$(RM) \
    $(TARGETDIR)/main.o \
    $(TARGETDIR)/Genetic.o \
    $(TARGETDIR)/Individual.o \
    $(TARGETDIR)/LocalSearch.o \
    $(TARGETDIR)/Params.o \
    $(TARGETDIR)/Population.o \
    $(TARGETDIR)/genvrp \
    $(TARGETDIR)/Split.o
     