
NAME=util
proc=main
mlib=func

CC=gcc
#STRIP=strip
RM=rm

CFLAG =-std=gnu99 -g -Wall
INC=/usr/include

MACHINE := $(shell uname -m)
ifeq ($(MACHINE), x86_64)
	LIB_DIR = /usr/lib
else
	LIB_DIR = /usr/lib32
endif

$(NAME): $(mlib).o $(proc).o
	$(CC) -o $(NAME) $(mlib).o $(proc).o
$(proc).o: $(proc).c
	$(CC) -c $(proc).c $(CFLAG) -I$(INC)
$(mlib).o: $(mlib).c
	$(CC) -c $(mlib).c $(CFLAG) -I$(INC)

clean:
	$(RM) -f *.o $(NAME)
