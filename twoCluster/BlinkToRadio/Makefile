COMPONENT=BlinkToRadioAppC
BUILD_EXTRA_DEPS=BlinkToRadioMsg.class
BlinkToRadioMsg.class: BlinkToRadioMsg.java
	javac BlinkToRadioMsg.java
BlinkToRadioMsg.java:
	mig java -target=null -java-classname=BlinkToRadioMsg BlinkToRadio.h BlinkToRadioMsg -o $@

CFLAGS += "-DCC2420_DEF_RFPOWER=6"

include $(MAKERULES)

