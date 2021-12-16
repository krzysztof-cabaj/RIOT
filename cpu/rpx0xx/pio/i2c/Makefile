MODULE = pio_i2c

include $(RIOTBASE)/Makefile.base

i2c.c: i2c.pio.h i2c_set_scl_sda.pio.h

i2c.pio.h: i2c.pio
	$(Q) $(RIOTTOOLS)/pioasm/pioasm -o RIOT $< $@

i2c_set_scl_sda.pio.h: i2c_set_scl_sda.pio
	$(Q) $(RIOTTOOLS)/pioasm/pioasm -o RIOT $< $@
