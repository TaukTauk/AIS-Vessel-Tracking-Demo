CC      = gcc
CFLAGS  = -std=c11 -Wall -Wextra -Wpedantic -O2
LDFLAGS = -lm

SRC_DIR = src
OBJ_DIR = objs
TEST_DIR = tests

SRCS = $(SRC_DIR)/ais.c \
       $(SRC_DIR)/vessel.c \
       $(SRC_DIR)/output.c \
       $(SRC_DIR)/main.c

OBJS = $(patsubst $(SRC_DIR)/%.c,$(OBJ_DIR)/%.o,$(SRCS))
BIN  = ais_demo

TEST_SRCS = $(TEST_DIR)/test_all.c \
            $(SRC_DIR)/ais.c \
            $(SRC_DIR)/vessel.c
TEST_BIN  = run_tests

.PHONY: all run test clean

all: $(BIN)

$(BIN): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c | $(OBJ_DIR)
	$(CC) $(CFLAGS) -c -o $@ $<

run: $(BIN)
	./$(BIN)

test: $(TEST_BIN)
	./$(TEST_BIN)

$(TEST_BIN): $(TEST_SRCS)
	$(CC) $(CFLAGS) -I$(SRC_DIR) -o $@ $^ $(LDFLAGS)

clean:
	rm -rf $(OBJ_DIR) $(BIN) $(TEST_BIN) sample.nmea map.html
