##########################################################################################
##### SimCore/RISC-V since 2018-07-05                ArchLab. TokyoTech              #####
##########################################################################################

CXX       = gcc
CXXFLAGS  = -O2 -Wall
TARGET    = initmem_gen
OBJS      = main.o
HEAD      = 

.SUFFIXES:
.SUFFIXES: .o .c
.PHONY: all clean run

##########################################################################################
all: $(TARGET)
	$(MAKE) $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

$(OBJS): Makefile $(HEAD)

.c.o:
	$(CXX) $(CXXFLAGS) -o $@ -c $<
##########################################################################################
run: ${OBJS}
	./initmem_gen bbl.bin root.bin devicetree.dtb

##########################################################################################
wc:
	wc -l *.c
clean:
	rm -f *.o initmem_gen* *out
	rm -f *.bin *.dtb *.txt bbl nuttx

##########################################################################################
