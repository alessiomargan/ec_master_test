#! /bin/sh
valgrind --tool=memcheck --trace-children=no --track-origins=yes  --leak-check=full  --show-leak-kinds=all $@

