all: installed

SVN_DIR = build/artoolkit-svn
SVN_URL = https://artoolkit.svn.sourceforge.net/svnroot/artoolkit/trunk/artoolkit
SVN_REVISION = -r 311
include $(shell rospack find mk)/svn_checkout.mk

installed: $(SVN_DIR) patched
	cp patch_v4l $(SVN_DIR)/Configure
	#Build Artoolkit with GStreamer change ligne below with: cd $(SVN_DIR) && ./Configure 5 y y y 
	#Currently build with V4L
	cd $(SVN_DIR) && ./Configure 1 n n n
	cd $(SVN_DIR) && make
	mkdir -p $(CURDIR)/lib
	cp -r $(CURDIR)/$(SVN_DIR)/lib/* $(CURDIR)/lib/
	mkdir -p $(CURDIR)/include
	cp -r $(CURDIR)/$(SVN_DIR)/include/* $(CURDIR)/include/
	touch installed

clean:
	-cd $(SVN_DIR) && make clean
	rm -rf $(CURDIR)/lib $(CURDIR)/include stage installed patched 

wipe: clean
	rm -rf $(SVN_DIR)
