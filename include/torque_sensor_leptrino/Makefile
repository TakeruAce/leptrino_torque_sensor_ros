EXEC = CFS_Sample
OBJS = rs_comm.o main.o

CC = g++

all:$(EXEC)

$(EXEC): $(OBJS)
	$(CC) -o $@ $(OBJS)

clean:
	-rm -f $(EXEC) *.elf *.gdb *.o

%.o: %.c
	$(CC) -c $(CFLAGS) -o $@ $< 