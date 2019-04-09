COMPONENT=BaseStationC
CFLAGS += -I$(TOSDIR)/lib/printf

BUILD_EXTRA_DEPS = BlinkToRadioMsg.class
CLEAN_EXTRA = *.class BlinkToRadioMsg.java

BlinkToRadioMsg.class: $(wildcard *.java) BlinkToRadioMsg.java
		javac BlinkToRadioMsg.java

BlinkToRadioMsg.java:
		mig java -target=null -java-classname=BlinkToRadioMsg BlinkToRadio.h BlinkToRadioMsg -o $@

include $(MAKERULES)

# COMPONENT=BaseStationC

# PFLAGS += -I$(TOSROOT)/tos/lib/printf
# PFLAGS += -DNEW_PRINTF_SEMANTICS
# CFLAGS += -DPRINTF_BUFFER_SIZE=300

# #BUILD_EXTRA_DEPS = BlinkToRadioMsg.class
# #CLEAN_EXTRA = *.class BlinkToRadioMsg.java

# # BlinkToRadioMsg.class: $(wildcard *.java) BlinkToRadioMsg.java
# # 		javac BlinkToRadioMsg.java

# # BlinkToRadioMsg.java:
# # 		mig java -target=null -java-classname=BlinkToRadioMsg BlinkToRadio.h BlinkToRadioMsg -o $@

# include $(MAKERULES)