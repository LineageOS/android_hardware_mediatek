#!/vendor/bin/sh

default_fact_prop=$(getprop ro.soc.manufacturer)

if [ "$default_fact_prop" == "" ]; then
	fact_val=$(cat /sys/devices/soc0/family)
	/vendor/bin/setprop ro.vendor.soc.manufacturer ${fact_val}
fi

default_soc_prop=$(getprop ro.soc.model)

if [ "$default_soc_prop" == "" ]; then
	soc_val=$(cat /sys/devices/soc0/soc_id)
	/vendor/bin/setprop ro.vendor.soc.model ${soc_val}
else
	soc_val="$default_soc_prop"
fi

soc_ext_val=$(cat /proc/device-tree/model-external-name 2>/dev/null)
if [ -z "$soc_ext_val" ]; then
	soc_ext_val=${soc_val}
fi
/vendor/bin/setprop ro.vendor.soc.model.external_name ${soc_ext_val}

soc_part_val=$(cat /proc/device-tree/model-part-name 2>/dev/null)
if [ -z "$soc_part_val" ]; then
    soc_part_val=${soc_val}
fi
/vendor/bin/setprop ro.vendor.soc.model.part_name ${soc_part_val}

/vendor/bin/setprop ro.vendor.soc.model_ready 1
