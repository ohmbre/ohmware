################################################################################
#
# emacs
#
################################################################################

MG_AUTORECONF = YES
MG_VERSION = master
MG_SITE = $(call github,troglobit,mg,$(MG_VERSION))
#MG_SITE_METHOD = git
MG_INSTALL_STAGING = YES
#MG_CONF_OPTS = --without-all --with-x=no
MG_DEPENDENCIES = ncurses libite

$(eval $(autotools-package))
