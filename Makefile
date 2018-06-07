# --- SYSTEM ---

SYSTEM     = x86-64_linux
LIBFORMAT  = static_pic

# --- DIRECTORIES ---

CXX = g++ -std=c++11
#BASISILOG = /opt/ibm/ILOG/CPLEX_Studio127
#CPOPTDIR   = $(BASISILOG)/cpoptimizer
#CONCERTDIR = $(BASISILOG)/concert
#CPLEXDIR   = $(BASISILOG)/cplex
BOOSTDIR   = /usr0/home/jmogali/Downloads/boost_1_64_0

# --- FLAGS ---

CCOPT = -m64 -fPIC -fno-strict-aliasing -fexceptions -DIL_STD
#CPLEXLIBDIR   = $(CPLEXDIR)/lib/$(SYSTEM)/$(LIBFORMAT)
#CONCERTLIBDIR = $(CONCERTDIR)/lib/$(SYSTEM)/$(LIBFORMAT)
#CPOPTLIBDIR = $(CPOPTDIR)/lib/$(SYSTEM)/$(LIBFORMAT)
BOOSTLIBDIR = $(BOOSTDIR)/lib

#CCLNFLAGS = -L$(CPLEXLIBDIR) -lilocplex -lcplex -L$(CONCERTLIBDIR) -lconcert -lm -pthread
#CLNFLAGS  = -L$(CPLEXLIBDIR) -lcplex -lm -pthread

#CONCERTINCDIR = $(CONCERTDIR)/include
#CPLEXINCDIR   = $(CPLEXDIR)/include
#CPOPTINCDIR   = $(CPOPTDIR)/include
CINCDIR       := $(addprefix -I,$(shell find include -type d))


# --- OPTIMIZATION FLAGS ---

DEBUG_OPT = -DNDEBUG -O3
#DEBUG_OPT = -g3 -O0
#PROF = -pg
PROF =

#CFLAGS += $(CCOPT) -I$(CPLEXINCDIR) -I$(CONCERTINCDIR) -I$(CPOPTINCDIR) $(CINCDIR) $(DEBUG_OPT) -I$(BOOSTDIR) -c $(PROF)
CFLAGS += $(CCOPT) $(CINCDIR) $(DEBUG_OPT) -I$(BOOSTDIR) -c $(PROF)
CFLAGS += -Wno-deprecated-declarations
ADD_CFLAGS = -fpermissive

LDFLAGS = -L$(BOOSTLIBDIR)

# ---- COMPILE  ----
SRC_DIR   := src
OBJ_DIR   := obj

SRC_DIRS  := $(shell find $(SRC_DIR) -type d)
OBJ_DIRS  := $(addprefix $(OBJ_DIR)/,$(SRC_DIRS))

SOURCES   := $(shell find $(SRC_DIR) -name '*.cpp' -o -name '*.c')
OBJ_FILES := $(addprefix $(OBJ_DIR)/, $(SOURCES))
OBJ_FILES := $(OBJ_FILES:.c=.o)
OBJ_FILES := $(OBJ_FILES:.cpp=.o)

#vpath %.c %.cpp $(SRC_DIRS)

# ---- TARGETS ----

EXECUTABLE=Boeing

all:$(EXECUTABLE)	

$(EXECUTABLE): makedir $(SOURCES) $(OBJ_FILES) 
	$(CXX) $(OBJ_FILES) $(LDFLAGS) $(PROF) -o $@

$(OBJ_DIR)/%.o: %.c
	$(CXX) $(CFLAGS) $(ADD_CFLAGS) $< -o $@
	
$(OBJ_DIR)/%.o: %.cpp
	$(CXX) $(CFLAGS) $< -o $@

makedir: $(OBJ_DIRS)

$(OBJ_DIRS):
	@mkdir -p $@

clean:
	@rm -rf obj 
	@rm -rf $(EXECUTABLE)
