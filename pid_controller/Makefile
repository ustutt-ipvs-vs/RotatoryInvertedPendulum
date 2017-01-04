CC=gcc

CFLAGS=-c -Wall -std=gnu99 -D_XOPEN_SOURCE=500 -D_GNU_SOURCE #-DDEBUG
LDFLAGS=-lm -lbcm2835

pid-controller: kalmanfilter.o pid_ctrl.o pid-controller.o
	$(CC) kalmanfilter.o pid_ctrl.o pid-controller.o $(LDFLAGS) -o $@

.PHONY: clean
clean:
	rm -rf pid.controller.o kalmanfilter.o pid_ctrl.o pid-controller
