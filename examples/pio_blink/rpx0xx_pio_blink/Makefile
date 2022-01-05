include $(RIOTBASE)/Makefile.base

blink.c: blink.hex.pio.h

blink.hex.pio.h: blink.pio
	$(Q) $(RIOTTOOLS)/pioasm/pioasm -o RIOT $< $@

