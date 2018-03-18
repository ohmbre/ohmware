PYTHON_UVLOOP_VERSION = master
PYTHON_UVLOOP_SITE = https://github.com/MagicStack/uvloop.git
PYTHON_UVLOOP_SITE_METHOD = git
#PYTHON_UVLOOP_GIT_SUBMODULES = YES
PYTHON_UVLOOP_SETUP_TYPE = setuptools
#PYTHON_FOO_BUILD_OPTS = --use-system-libuv

#define PYTHON_UVLOOP_BUILDEXT
#	cd $(BUILD_DIR)/python-uvloop-$(PYTHON_UVLOOP_VERSION) && $(PKG_PYTHON_SETUPTOOLS_ENV) $(HOST_DIR)/bin/python setup.py build_ext --use-system-libuv
#endef
#
#PYTHON_UVLOOP_PRE_BUILD_HOOKS += PYTHON_UVLOOP_BUILDEXT

$(eval $(python-package))
