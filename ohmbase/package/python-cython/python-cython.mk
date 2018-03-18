PYTHON_CYTHON_VERSION = master
PYTHON_CYTHON_SITE = https://github.com/cython/cython.git
PYTHON_CYTHON_SITE_METHOD = git
PYTHON_CYTHON_SETUP_TYPE = setuptools

$(eval $(host-python-package))
