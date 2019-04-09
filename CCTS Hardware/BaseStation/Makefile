COMPONENT=BaseStationC
CFLAGS += -I$(TOSDIR)/lib/printf

BUILD_EXTRA_DEPS = BlinkToRadioMsg.class
CLEAN_EXTRA = *.class BlinkToRadioMsg.java

BlinkToRadioMsg.class: $(wildcard *.java) BlinkToRadioMsg.java
		javac BlinkToRadioMsg.java

BlinkToRadioMsg.java:
		mig java -target=null -java-classname=BlinkToRadioMsg BlinkToRadio.h BlinkToRadioMsg -o $@

include $(MAKERULES)