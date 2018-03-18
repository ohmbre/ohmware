define PYTHON_SIP_BUMPVER
	wget "http://excellmedia.dl.sourceforge.net/project/pyqt/sip/sip-4.19.7/sip-4.19.7.tar.gz" -O $(BR2_DL_DIR)/sip-4.19.7.tar.gz
	rm -rf $(BUILD_DIR)/python-sip-4.18
	tar xvf $(BR2_DL_DIR)/sip-4.19.7.tar.gz
	mv sip-4.19.7 $(BUILD_DIR)/python-sip-4.18
	touch $(BUILD_DIR)/python-sip-4.18/.stamp_downloaded
endef

PYTHON_SIP_POST_EXTRACT_HOOKS += PYTHON_SIP_BUMPVER

define HOST_PYTHON_SIP_BUMPVER
	wget "http://excellmedia.dl.sourceforge.net/project/pyqt/sip/sip-4.19.7/sip-4.19.7.tar.gz" -O $(BR2_DL_DIR)/sip-4.19.7.tar.gz
	rm -rf $(BUILD_DIR)/host-python-sip-4.18
	tar xvf $(BR2_DL_DIR)/sip-4.19.7.tar.gz
	mv sip-4.19.7 $(BUILD_DIR)/host-python-sip-4.18
	touch $(BUILD_DIR)/host-python-sip-4.18/.stamp_downloaded
endef

HOST_PYTHON_SIP_POST_EXTRACT_HOOKS += HOST_PYTHON_SIP_BUMPVER


