#
# build using configure as best we can, should work for most targets
# David McCullough <davidm@snapgear.com>
#
#

all: build
	$(MAKE) -C build

build: Makefile
	rm -rf build
	mkdir build
	chmod u+x configure
	( \
		cd build; \
		export CC="$(CC) $(CFLAGS)"; \
		export CXX="$(CXX) $(CFLAGS)"; \
		export LDFLAGS="$(LDFLAGS)"; \
		export LIBS="$(LDLIBS)"; \
		sh ../configure $(CONFIGURE_OPTS) --with-flexlib=../../../../flex/build/libfl.a --enable-security-context=no --prefix= ; \
	)

romfs:
	$(ROMFSINST) build/src/racoon/racoon /bin/racoon
	$(ROMFSINST) build/src/racoon/racoonctl /bin/racoonctl
	$(ROMFSINST) build/src/setkey/setkey /bin/setkey

clean:
	rm -rf build

