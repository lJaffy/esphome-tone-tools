# util/Makefile — build chime core to WASM with emsdk 3.1.6
# Commits artifacts to wasm/chime.js + wasm/chime.wasm for web serving.
# Requires emsdk 3.1.6 (https://github.com/emscripten-core/emsdk).
# Usage:
#   make            # build (auto-installs emsdk 3.1.6 under /tmp/emsdk if missing)
#   make clean

EMSDK_VERSION := 3.1.6
EMSDK_DIR ?= /tmp/emsdk
EMCC := /usr/bin/emcc
CORE_SRC := ../esphome/components/chime/chime_pattern.cpp ../esphome/components/chime/chime_engine.cpp
OUT_DIR := ../wasm
OUT_JS := $(OUT_DIR)/chime.js
OUT_WASM := $(OUT_DIR)/chime.wasm

EMCC_FLAGS := -O3 -std=c++17 -I../esphome/components/chime \
	-s WASM=1 -s ALLOW_MEMORY_GROWTH=1 -s MODULARIZE=1 -s EXPORT_NAME=ChimeWasm \
	-s EXPORTED_RUNTIME_METHODS='["cwrap","getValue","setValue","HEAPF32"]' \
	--bind -s ENVIRONMENT=web -s SINGLE_FILE=0

.PHONY: all clean install-emsdk

all: $(OUT_JS)

$(OUT_JS): bindings.cpp $(CORE_SRC)
	@if [ ! -f "$(EMCC)" ]; then echo "emsdk $(EMSDK_VERSION) not found at $(EMSDK_DIR), running make install-emsdk..."; $(MAKE) install-emsdk; fi
	. $(EMSDK_DIR)/emsdk_env.sh && $(EMCC) $(EMCC_FLAGS) $(CORE_SRC) bindings.cpp -o $(OUT_JS)
	@echo "Built $(OUT_JS) + $(OUT_WASM) (emsdk $(EMSDK_VERSION))"

install-emsdk:
	@if [ -d "$(EMSDK_DIR)" ]; then echo "emsdk dir exists at $(EMSDK_DIR)"; else git clone https://github.com/emscripten-core/emsdk.git $(EMSDK_DIR); fi
	cd $(EMSDK_DIR) && ./emsdk install $(EMSDK_VERSION) && ./emsdk activate $(EMSDK_VERSION)
	@echo "emsdk $(EMSDK_VERSION) installed at $(EMSDK_DIR). Run: source $(EMSDK_DIR)/emsdk_env.sh"

clean:
	rm -f $(OUT_JS) $(OUT_WASM)
