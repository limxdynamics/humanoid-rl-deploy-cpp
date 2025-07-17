SHELL := /bin/bash

all:
	@source $(PWD)/ament/setup.bash && colcon --log-base $(PWD)/build/log \
	    build \
	    --merge-install \
	    --build-base $(PWD)/build/build \
	    --install-base $(PWD)/build/install \
	    --cmake-force-configure \
	    --cmake-args \
	        -DCMAKE_BUILD_TYPE=Release \
	        -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
	        --no-warn-unused-cli
	@sed -i '/unset COLCON_CURRENT_PREFIX/d' $(PWD)/build/install/setup.bash
	@echo 'export LIMX_ABILITY_ROOT_PATH=$$COLCON_CURRENT_PREFIX' >> $(PWD)/build/install/setup.bash
	@echo 'export LIMX_ABILITY_LIB_PATH=$$COLCON_CURRENT_PREFIX/lib' >> $(PWD)/build/install/setup.bash
	@echo 'export LIMX_ABILITY_ETC_PATH=$$COLCON_CURRENT_PREFIX/etc' >> $(PWD)/build/install/setup.bash
	@echo 'export LIMX_ABILITY_BIN_PATH=$$COLCON_CURRENT_PREFIX/bin' >> $(PWD)/build/install/setup.bash
	@echo 'unset COLCON_CURRENT_PREFIX' >> $(PWD)/build/install/setup.bash
	@sed -i '/unset COLCON_CURRENT_PREFIX/d' $(PWD)/build/install/setup.sh
	@echo 'export LIMX_ABILITY_ROOT_PATH=$$COLCON_CURRENT_PREFIX' >> $(PWD)/build/install/setup.sh
	@echo 'export LIMX_ABILITY_LIB_PATH=$$COLCON_CURRENT_PREFIX/lib' >> $(PWD)/build/install/setup.sh
	@echo 'export LIMX_ABILITY_ETC_PATH=$$COLCON_CURRENT_PREFIX/etc' >> $(PWD)/build/install/setup.sh
	@echo 'export LIMX_ABILITY_BIN_PATH=$$COLCON_CURRENT_PREFIX/bin' >> $(PWD)/build/install/setup.sh
	@echo 'unset COLCON_CURRENT_PREFIX' >> $(PWD)/build/install/setup.sh
	@sed -i '/unset COLCON_CURRENT_PREFIX/d' $(PWD)/build/install/setup.zsh
	@echo 'export LIMX_ABILITY_ROOT_PATH=$$COLCON_CURRENT_PREFIX' >> $(PWD)/build/install/setup.zsh
	@echo 'export LIMX_ABILITY_LIB_PATH=$$COLCON_CURRENT_PREFIX/lib' >> $(PWD)/build/install/setup.zsh
	@echo 'export LIMX_ABILITY_ETC_PATH=$$COLCON_CURRENT_PREFIX/etc' >> $(PWD)/build/install/setup.zsh
	@echo 'export LIMX_ABILITY_BIN_PATH=$$COLCON_CURRENT_PREFIX/bin' >> $(PWD)/build/install/setup.zsh
	@echo 'unset COLCON_CURRENT_PREFIX' >> $(PWD)/build/install/setup.zsh

clean:
	@rm -rf $(PWD)/build
