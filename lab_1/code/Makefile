a.out: requirements
	echo '#!/bin/sh' > $@
	echo 'python3 main.py' >> $@
	echo 'exit $$?' >> $@
	chmod +x $@
.PHONY: a.out

requirements:
	if -e ./requirements.txt ; then pip install -r ./requirements.txt ; fi
.PHONY: requirements

clean:
	rm -f a.out
.PHONY: clean
