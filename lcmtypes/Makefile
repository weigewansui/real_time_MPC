LCM_JAR = /usr/local/share/java/lcm.jar
SPACE = lcmtypes
LCMTYPES_NAMES := $(shell ls *.lcm)
LCMTYPES_C := $(LCMTYPES_NAMES:%.lcm=%.c)
LCMTYPES_O := $(LCMTYPES_C:%.c=%.o)
LCMTYPES_HPP := $(LCMTYPES_NAMES:%.lcm=%.hpp)
LCMTYPES_JAVA := $(LCMTYPES_C:%.c=%.java)

all: $(LCMTYPES_C) $(LCMTYPES_O) $(LCMTYPES_HPP) $(LCMTYPES_JAVA) lcmtypes.jar
# all: $(LCMTYPES_JAVA) lcmtypes.jar
	
%.c: %.lcm
	lcm-gen -c *.lcm

%.hpp:
	lcm-gen -x *.lcm

%.java: %.lcm
	lcm-gen -j *.lcm

lcmtypes.jar:
	javac -cp $(LCM_JAR) $(SPACE)/*.java
	jar cf lcmtypes.jar $(SPACE)/*.class

clean:
	rm -f *.h *.c *.jar *.o *.hpp
	rm -rf lcmtypes/
