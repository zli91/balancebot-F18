all:                                                         
	@make -C lcmtypes
	@make -C balancebot --no-print-directory
	@make -C measure_motors --no-print-directory
	@make -C test_motors --no-print-directory                                                       

lcmtypes:
	@make -C lcmtypes

lcmspy:	
	/bin/bash setenv.sh
	@make -C lcmtypes                                                          
	@make -C java 

clean:                                             
	@make -C balancebot -s clean
	@make -C measure_motors -s clean
	@make -C test_motors -s clean
	@make -C lcmtypes -s clean                                                
	@make -C java -s clean
