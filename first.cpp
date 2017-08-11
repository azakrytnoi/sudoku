/*
 * first.cpp
 *
 *  Created on: Aug 11, 2017
 *      Author: aleksey
 */

#include <iostream>
#include <fstream>
#include <cmath>

int field[9][9];

bool row(int i, int j, int x) {
	for (int g = 0; g < 9; g++) {
		if (field[i][g] == x) {
			return false;
		}
	}
	return true;
}

bool column(int i, int j, int x) {
	for (int g = 0; g < 9; g++) {
		if (field[g][j] == x) {
			return false;
		}
	}
	return true;
}

bool square(int i, int j, int x) {
	int r = std::ceil((i + 1) / 3.);
	int c = std::ceil((j + 1) / 3.);
	for (int g = (r - 1) * 3; g < (r - 1) * 3 + 3; g++) {
		for (int h = (c - 1) * 3; h < (c - 1) * 3 + 3; h++) {
			if (field[g][h] == x) {
				return false;
			}
		}
	}
	return true;
}

int main() {
	int un(0), tmp;
	bool b(true);
	{
		const char* filename = "sudoku.txt";
		std::ifstream in(filename);
		if (!in) {
			std::cout << "can't open" << std::endl;
			return 0;
		}
		for (int i = 0; i < 9; i++) {
			for (int j = 0; j < 9; j++) {
				in >> field[i][j];
				if (!field[i][j]) {
					un++;
				}
			}
		}
	}

	while (un && b) {
		b = false;
		for (int i = 0; i < 9; i++) {
			for (int j = 0; j < 9; j++) {
				if (field[i][j]) {
					continue;
				}
				tmp = 0;
				for (int x = 1; x < 10; x++) {
					if (row(i, j, x) && column(i, j, x) && square(i, j, x)) {
						if (!tmp) {
							tmp = x;
						} else {
							tmp = 0;
							break;
						}
					}
				}
				if (tmp) {
					field[i][j] = tmp;
					b = true;
					un--;
				}
			}
		}
	}
	if (!b) {
		std::cout << "can't be solved" << std::endl;
	} else {
		std::cout << "solved" << std::endl;
	}

	for (int i = 0; i < 9; i++) {
		for (int j = 0; j < 9; j++) {
			std::cout << field[i][j] << ' ';
		}
		std::cout << std::endl;
	}
}
