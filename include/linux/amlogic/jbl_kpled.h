
#ifndef __INCLUDE_JBL_KPLED_H
#define __INCLUDE_JBL_KPLED_H

struct aml_kpled_platform_data {
	void (*kpled_init)(void);
	void (*power_on_kpled)(void);
	void (*power_off_kpled)(void);
	unsigned (*get_kpled_level)(void);
	void (*set_kpled_level)(unsigned);
	int max_brightness;
	int dft_brightness;
};

#endif

