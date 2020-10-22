all:
	@echo '## Make commands ##'
	@echo
	@$(MAKE) -pRrq -f $(lastword $(MAKEFILE_LIST)) : 2>/dev/null | awk -v RS= -F: '/^# File/,/^# Finished Make data base/ {if ($$1 !~ "^[#.]") {print $$1}}' | sort | egrep -v -e '^[^[:alnum:]]' -e '^$@$$' | xargs

install_anaconda:
	@Makefile.scripts/install_anaconda.sh

install: install_anaconda
	@Makefile.scripts/install.sh

lint:
	@Makefile.scripts/lint.sh

test:
	@Makefile.scripts/test.sh
