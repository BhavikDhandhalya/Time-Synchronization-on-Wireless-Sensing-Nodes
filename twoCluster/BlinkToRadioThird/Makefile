COMPONENT=BlinkToRadioThirdAppC
BUILD_EXTRA_DEPS=BlinkToRadioMsg.class
BlinkToRadioMsg.class: BlinkToRadioMsg.java
	javac BlinkToRadioMsg.java
BlinkToRadioMsg.java:
	mig java -target=null -java-classname=BlinkToRadioMsg BlinkToRadio.h BlinkToRadioMsg -o $@



include $(MAKERULES)

