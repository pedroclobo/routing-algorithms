TARGETS = bin/dv-simulator bin/dvrpp-simulator bin/pv-simulator bin/ls-simulator

CC = g++
CFLAGS = -Wall -Werror --pedantic -O0 -g
LD = g++
LDFLAGS =

default: $(TARGETS)

bin/dv-simulator: bin/dv.o bin/routing-simulator.o
bin/dvrpp-simulator: bin/dvrpp.o bin/routing-simulator.o
bin/pv-simulator: bin/pv.o bin/routing-simulator.o
bin/ls-simulator: bin/ls.o bin/routing-simulator.o

$(TARGETS):
	$(LD) $(LDFLAGS) -o $@ $^

bin/%.o: src/%.cpp
	$(CC) -MT $@ -MMD -MP -MF $@.d $(CFLAGS) -c -o $@ $<

bin/%.o: src/%.c
	$(CC) -MT $@ -MMD -MP -MF $@.d $(CFLAGS) -c -o $@ $<

clean:
	(rm -f bin/*)

-include $(wildcard *.d)
