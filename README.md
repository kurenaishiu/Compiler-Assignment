# Project Assignment 

**Introduction to Compiler Design by Prof. Yi-Ping You**

## Assignment

In order to keep this assignment simple, only the `integer` type is needed to be implemented and the `array` type is not considered. This assignment is to generate `RISC-V` instructions for a `P` program that contains any of the following constructs:

- Global variable or local variable declaration.
- Global constant or local constant declaration.
- Function declaration.
- Assign statement.
- Simple statement.
- Expressions with only `+` `-` (unary and binary) `*` `/` `mod` `function invocation` included.
- Function invocation statement.
- Conditional statement.
- For statement and while statement.

The generated `RISC-V` instructions would be saved in a file with the same name as the input `P` file but with a `.S` extension. In addition, the file would be stored in a directory, which is set by the flag `--save-path [save path]`. For example, the following command translates `./test.p` into `../test/riscv/test.S`.

```sh
./compiler test.p --save-path ../test/riscv
```

## Generating RISC-V Instructions

We use a simple computation model called [**stack machine**](https://en.wikipedia.org/wiki/Stack_machine).

- When traversing to a `variable reference` node, push the **value** of the variable on the stack if it appears on the `RHS` of the `assignment` node, or push the **address** of the variable to the stack if it's on the `LHS` of the `assignment` node.

- When traversing to a `computation` node, (1) pop the values on the stack to some registers, and (2) then compute the result with one or more instructions, and (3) finally push the result back to the stack.

- When traversing to an `assignment` node, (1) pop the value and the address on the stack to some registers, and (2) then store the value to that address.

- For more precise steps, see [simple compilers](https://en.wikipedia.org/wiki/Stack_machine#Simple_compilers).

---
## Project Structure

- `README.md`
- /src
  - Makefile
  - `scanner.l`
  - `parser.y`
  - /include
    - /AST
    - /semantic
    - /visitor
    - /codegen
      - CodeGenerator.hpp - for code generation in visitor pattern version
  - /lib
    - /AST
    - /semantic
    - /visitor
    - /codegen
      - CodeGenerator.cpp - for code generation in visitor pattern version
  
- /report
  - `README.md`

## Build and Execute

> [!important]
> If you're on macOS, please use the emulator exclusively for testing your homework.

- Get docker image: `make docker-pull`
- Activate docker environment: `./activate_docker.sh`
- Build: `make`
- Execute: `./compiler [input file] --save-path [save path]`
- Test: `make test`
- Test on board: `make board`