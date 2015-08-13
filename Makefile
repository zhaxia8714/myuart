OBJECTS := myuart
all:
	$(CC) -g -o $(OBJECTS) -I./ main.c  $(LDFLAGS)

.PHONY: clean
clean:
	@rm -f  $(OBJECTS) *~ $(OBJECTS)

