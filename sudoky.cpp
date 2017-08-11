/*
 * first.cpp
 *
 *  Created on: Aug 11, 2017
 *      Author: aleksey
 */

#include <iostream>
#include <fstream>
#include <cmath>
#include <array>
#include <algorithm>

class sudoky {
private:
	std::array<std::array<int, 9>, 9> _field;
	int _gaps;

	bool row(int i, int guess);
	bool col(int j, int guess);
	bool square(int i, int j, int guess);
public:
	sudoky() :
			_gaps(0) {
	}

	friend std::ostream& operator <<(std::ostream& out, const sudoky& s);
	friend std::istream& operator >>(std::istream& in, sudoky& s);

	friend class solver;
};

class solver {
public:

	static bool solve(sudoky& board);
};

bool sudoky::row(int i, int guess) {
	for (int col = 0; col < 9; col++) {
		if (_field[i][col] == guess) {
			return false;
		}
	}
	return true;
}

bool sudoky::col(int j, int guess) {
	for (int row = 0; row < 9; row++) {
		if (_field[row][j] == guess) {
			return false;
		}
	}
	return true;
}

bool sudoky::square(int i, int j, int guess) {
	int row = std::ceil((i + 1) / 3.);
	int col = std::ceil((j + 1) / 3.);
	for (int ridx = (row - 1) * 3; ridx < (row - 1) * 3 + 3; ridx++) {
		for (int cidx = (col - 1) * 3; cidx < (col - 1) * 3 + 3; cidx++) {
			if (_field[ridx][cidx] == guess) {
				return false;
			}
		}
	}
	return true;
}

bool solver::solve(sudoky& board) {
	bool placed(true);
	while (board._gaps && placed) {
		placed = false;
		int i(0);
		std::for_each(board._field.begin(), board._field.end(),
				[&](auto& row) {
					int j(0);
					std::for_each(row.begin(), row.end(), [&](auto& cell) {
								if (!cell) {
									int guess(0);
									for (int try_guess = 1; try_guess <= 9; try_guess++) {
										if (board.row(i, try_guess) && board.col(j, try_guess)
												&& board.square(i, j, try_guess)) {
											if (!guess) {
												guess = try_guess;
											} else {
												guess = 0;
												break;
											}
										}
									}
									if (guess) {
										cell = guess;
										board._gaps--;
										placed = true;
									}
								}
								j++;
							});
					i++;
				});
	}
	return placed;
}

std::ostream& operator <<(std::ostream& out, const sudoky& board) {
	std::for_each(board._field.begin(), board._field.end(), [&](auto& row) {
		std::for_each(row.begin(), row.end(), [&](auto& cell) {
					out << cell << ' ';
				});
		out << std::endl;
	});
	return out;
}

std::istream& operator >>(std::istream& in, sudoky& board) {
	board._gaps = 0;
	std::for_each(board._field.begin(), board._field.end(), [&](auto& row) {
		std::for_each(row.begin(), row.end(), [&](auto& cell) {
					in >> cell;
					if (!cell) {
						board._gaps++;
					}
				});
	});
	return in;
}

int main() {
	sudoky board;

	{
		const char* filename = "sudoku.txt";
		std::ifstream in(filename);
		if (!in) {
			std::cout << "can't open" << std::endl;
			return 0;
		}
		in >> board;
	}

	if (!solver::solve(board)) {
		std::cout << "can't be solved" << std::endl;
	} else {
		std::cout << "solved" << std::endl;
	}

	std::cout << board << std::endl;
}
