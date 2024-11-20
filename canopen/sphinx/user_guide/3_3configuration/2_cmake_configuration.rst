Cmake Configuration
====================



In order to build the configuration package and generate the necessary runtime artifacts from the
bus configuration file and eds/dcf files, the lely_core_libraries package contains an extra
CMAKE macro.

**cogen_dcf(target)**

*Target: the name of the configuration (e.g. for config/{bus_config_name_1} is bus_config_name_1)*

.. code-block::

  cogen_dcf(bus_config)