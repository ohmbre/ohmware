################################################################################
#
# emacs
#
################################################################################

EMACS_AUTORECONF = YES
EMACS_VERSION = 25.3
EMACS_SITE = http://ftp.gnu.org/gnu/emacs
EMACS_SOURCE = emacs-$(EMACS_VERSION).tar.gz
EMACS_INSTALL_STAGING = YES
EMACS_INSTALL_TARGET = YES
EMACS_CONF_OPTS = --without-all --with-x=no
EMACS_CONF_ENV = CANNOT_DUMP=yes
EMACS_DEPENDENCIES = ncurses libite
EMACS_MAKE = LD_LIBRARY_PATH=$(TARGET_DIR)/lib $(MAKE)

$(eval $(autotools-package))
