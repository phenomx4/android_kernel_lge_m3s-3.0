#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>

#include <mach/msm_battery.h>
#include <mach/board_lge.h>

#ifdef CONFIG_MACH_LGE_M3S
static u32 m3s_spg_batt_capacity(u32 current_soc);

static struct msm_psy_batt_pdata msm_psy_batt_data = {
	.voltage_min_design	= 3200,
	.voltage_max_design	= 4350,
	.avail_chg_sources		= AC_CHG | USB_CHG,
	.batt_technology		= POWER_SUPPLY_TECHNOLOGY_LION,
	.calculate_capacity		= &m3s_spg_batt_capacity,
};

static u32 m3s_spg_batt_capacity(u32 current_soc)
{
	if(current_soc > 100)
		current_soc = 100;

	return current_soc;
}
#endif

static struct platform_device msm_batt_device = {
	.name               = "msm-battery",
	.id                 = -1,
	.dev.platform_data  = &msm_psy_batt_data,
};

/* misc platform devices */
static struct platform_device *m3s_misc_devices[] __initdata = {
	&msm_batt_device,
};

/* main interface */
void __init lge_add_misc_devices(void)
{
	platform_add_devices(m3s_misc_devices, ARRAY_SIZE(m3s_misc_devices));

}

