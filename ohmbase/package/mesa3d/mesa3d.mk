
define FT8XX_PATCH_HOOK
	find $(MESA3D_SRCDIR) -type f -print0 | xargs -0 sed -i 's/pl111/ft8xx/g'
	find $(MESA3D_SRCDIR) -type f -print0 | xargs -0 sed -i 's/PL111/FT8XX/g'
	mv $(MESA3D_SRCDIR)/src/gallium/drivers/pl111 $(MESA3D_SRCDIR)/src/gallium/drivers/ft8xx
	mv $(MESA3D_SRCDIR)/src/gallium/winsys/pl111 $(MESA3D_SRCDIR)/src/gallium/winsys/ft8xx
	mv $(MESA3D_SRCDIR)/src/gallium/winsys/ft8xx/drm/pl111_drm_public.h $(MESA3D_SRCDIR)/src/gallium/winsys/ft8xx/drm/ft8xx_drm_public.h
	mv $(MESA3D_SRCDIR)/src/gallium/winsys/ft8xx/drm/pl111_drm_winsys.c $(MESA3D_SRCDIR)/src/gallium/winsys/ft8xx/drm/ft8xx_drm_winsys.c
endef

MESA3D_POST_PATCH_HOOKS += FT8XX_PATCH_HOOK
MESA3D_GALLIUM_DRIVERS-$(BR2_PACKAGE_MESA3D_GALLIUM_DRIVER_VC4) = ft8xx vc4 virgl
MESA3D_PLATFORMS += surfaceless
MESA3D_PYTHON_INTERPRETER=$(HOST_DIR)/usr/bin/python2.7
HOST_PYTHON_MAKO_NEEDS_HOST_PYTHON=python2
MESA3D_DEPENDENCIES += host-python-setuptools host-python-markupsafe host-python-mako

define MESA3D_USEMAKO_CONFIG_HOOK
        cd $(HOST_PYTHON_SETUPTOOLS_BUILDDIR) && \
		$(HOST_DIR)/usr/bin/python2.7 bootstrap.py && \
		$(HOST_DIR)/usr/bin/python2.7 setup.py install
	cd $(HOST_PYTHON_MARKUPSAFE_BUILDDIR) && \
		$(HOST_DIR)/usr/bin/python2.7 setup.py install
	cd $(HOST_PYTHON_MAKO_BUILDDIR) && $(HOST_DIR)/usr/bin/python2.7 setup.py install
endef

MESA3D_POST_CONFIGURE_HOOKS += MESA3D_USEMAKO_CONFIG_HOOK
