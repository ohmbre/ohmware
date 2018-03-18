################################################################################
#
# python-pyqt5
#
################################################################################

PYTHON_PYQTNEW_VERSION = 5.9.2
PYTHON_PYQTNEW_SOURCE = PyQt5_gpl-$(PYTHON_PYQTNEW_VERSION).tar.gz
PYTHON_PYQTNEW_SITE = http://downloads.sourceforge.net/project/pyqt/PyQt5/PyQt-$(PYTHON_PYQTNEW_VERSION)
PYTHON_PYQTNEW_LICENSE = GPL-3.0
PYTHON_PYQTNEW_LICENSE_FILES = LICENSE

PYTHON_PYQTNEW_DEPENDENCIES = python-sip host-python-sip qt5base

ifeq ($(BR2_PACKAGE_PYTHON),y)
PYTHON_PYQTNEW_PYTHON_DIR = python$(PYTHON_VERSION_MAJOR)
PYTHON_PYQTNEW_RM_PORT_BASE = port_v3
else ifeq ($(BR2_PACKAGE_PYTHON3),y)
PYTHON_PYQTNEW_PYTHON_DIR = python$(PYTHON3_VERSION_MAJOR)
PYTHON_PYQTNEW_RM_PORT_BASE = port_v2
endif

PYTHON_PYQTNEW_CONF_OPTS = \
	--bindir $(TARGET_DIR)/usr/bin \
	--destdir $(TARGET_DIR)/usr/lib/$(PYTHON_PYQTNEW_PYTHON_DIR)/site-packages \
	--qmake $(HOST_DIR)/bin/qmake \
	--sysroot $(STAGING_DIR)/usr \
	-w --confirm-license \
	--no-designer-plugin \
	--no-docstrings \
	--no-sip-files

define PYTHON_PYQTNEW_CONFIGURE_CMDS
	(cd $(@D); export LD_LIBRARY_PATH=$(HOST_DIR)/aarch64-ohm-linux-gnu/sysroot/usr/lib; \
		$(TARGET_MAKE_ENV) \
		$(TARGET_CONFIGURE_OPTS) \
		$(HOST_DIR)/bin/python configure.py \
			$(PYTHON_PYQTNEW_CONF_OPTS); unset LD_LIBRARY_PATH \
	)
endef

define PYTHON_PYQTNEW_BUILD_CMDS
	$(TARGET_MAKE_ENV) $(TARGET_CONFIGURE_OPTS) $(MAKE) CFLAGS='-I../../../host/aarch64-ohm-linux-gnu/sysroot/usr/include/qt5/QtPrintSupport' $(@D)
endef

# __init__.py is needed to import PyQt5
# __init__.pyc is needed if BR2_PACKAGE_PYTHON_PYC_ONLY is set
define PYTHON_PYQTNEW_INSTALL_TARGET_CMDS
	$(TARGET_MAKE_ENV) $(TARGET_CONFIGURE_OPTS) $(MAKE) -C $(@D) install
	touch $(TARGET_DIR)/usr/lib/$(PYTHON_PYQTNEW_PYTHON_DIR)/site-packages/PyQt5/__init__.py
	$(RM) -rf $(TARGET_DIR)/usr/lib/$(PYTHON_PYQTNEW_PYTHON_DIR)/site-packages/PyQt5/uic/$(PYTHON_PYQTNEW_RM_PORT_BASE)
endef

$(eval $(generic-package))
