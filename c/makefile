all: pngsym runpngsym

pngsym: pngsym.c
	gcc -o pngsym pngsym.c -lpng -lm

runpngsym: pngsym
	./pngsym ../references/png/*.png >../ESP32/main/symbols.h
