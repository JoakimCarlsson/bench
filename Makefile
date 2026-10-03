# Builds the four benchmark executables and runs the comparison.
#
#   make            build everything
#   make run        build, then run the interleaved comparison (run.py)
#   make clean
#
# Override the toolchains on the command line or in the environment:
#   make ZIG=/path/to/zig CC=clang CXX=clang++ CARGO=cargo

ifeq ($(origin CC),default)
CC := clang
endif
ifeq ($(origin CXX),default)
CXX := clang++
endif
ZIG ?= zig
CARGO ?= cargo
PYTHON ?= python

ifeq ($(OS),Windows_NT)
EXE := .exe
LIBM :=
else
EXE :=
LIBM := -lm -pthread
endif

OUT := out
CFLAGS   := -std=c11 -O3 -march=native -ffp-contract=off -Wall -Wextra
CXXFLAGS := -std=c++20 -O3 -march=native -ffp-contract=off -fno-exceptions -fno-rtti -Wall -Wextra

C_SRC   := $(wildcard c/src/*.c)
CPP_SRC := $(wildcard cpp/src/*.cpp)
ZIG_SRC := $(wildcard zig/src/*.zig) zig/build.zig
RUST_SRC := $(wildcard rust/src/*.rs) rust/Cargo.toml rust/.cargo/config.toml

.PHONY: all build run clean

all: build

build: $(OUT)/bench_c$(EXE) $(OUT)/bench_cpp$(EXE) $(OUT)/bench_zig$(EXE) $(OUT)/bench_rust$(EXE)

$(OUT):
	mkdir -p $(OUT)

$(OUT)/bench_c$(EXE): $(C_SRC) c/src/*.h | $(OUT)
	$(CC) $(CFLAGS) -o $@ $(C_SRC) $(LIBM)

$(OUT)/bench_cpp$(EXE): $(CPP_SRC) cpp/src/*.hpp | $(OUT)
	$(CXX) $(CXXFLAGS) -o $@ $(CPP_SRC) $(LIBM)

$(OUT)/bench_zig$(EXE): $(ZIG_SRC) | $(OUT)
	cd zig && $(ZIG) build --prefix ../$(OUT) --prefix-exe-dir .

$(OUT)/bench_rust$(EXE): $(RUST_SRC) | $(OUT)
	cd rust && $(CARGO) build --release
	cp rust/target/release/bench_rust$(EXE) $@

run: build
	$(PYTHON) run.py

clean:
	rm -rf $(OUT) zig/.zig-cache zig/zig-out rust/target
