.PHONY: run
run: clean
	mkdir -p globalregs
	$(SCANALYZE_DIR)/scanalyze.opt -noui alignring.tcl > output.log 2> err.log

.PHONY: clean
clean:
	rm -f *.xf
	rm -f *.xf.bu
	rm -f globalregs/*.gr

.PHONY: view
view:
	optirun $(SCANALYZE_DIR)/scanalyze.opt load.tcl > scanalyze_out.log

MANIFEST:
	python gen_manifest.py `ls -l *.ply | wc -l`
