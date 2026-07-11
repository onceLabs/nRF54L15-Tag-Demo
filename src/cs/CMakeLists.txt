target_sources_ifdef(CONFIG_APP_CS app PRIVATE
	cs_shared.c
	cs_initiator.c
	cs_reflector.c
	distance.c
)
zephyr_include_directories(.)
